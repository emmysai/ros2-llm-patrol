#!/usr/bin/env python3
import os
import json

import streamlit as st

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from langchain_google_genai import ChatGoogleGenerativeAI


SYSTEM_BASE = """
Du bist ein wissenschaftlicher ROS2 Robot Assistant.
Du bekommst reale Sensordaten aus einem TurtleBot3 in Gazebo.
Nutze ausschließlich die übergebenen Daten. Keine Annahmen über nicht vorhandene Sensoren (z.B. Batterie).
Keine Meta-Kommentare. Keine Erwähnung von Tools/JSON/Debug.
Antworte kurz, strukturiert und sachlich.
"""

SYSTEM_STATE_ONLY = SYSTEM_BASE + """
Aufgabe: Gib NUR den Roboterzustand + kurze Interpretation aus.

Format exakt so:

Zustand:
- ...

Interpretation:
- ...
"""

SYSTEM_WP_ONLY = SYSTEM_BASE + """
Aufgabe: Gib NUR den naechsten Waypoint aus (Name + Distanz).

Format exakt so:

Naechster Waypoint:
- ...
"""

SYSTEM_BOTH = SYSTEM_BASE + """
Aufgabe: Gib Roboterzustand + Interpretation + naechsten Waypoint aus.

Format exakt so:

Zustand:
- ...

Interpretation:
- ...

Naechster Waypoint:
- ...
"""


def model_name() -> str:
    return os.getenv("GEMINI_MODEL", "gemini-2.5-flash")


@st.cache_resource
def ros_clients():
    rclpy.init(args=None)
    node = Node("llm_streamlit_ui_neu")
    state_cli = node.create_client(Trigger, "/llm_tools/get_robot_state")
    nearest_cli = node.create_client(Trigger, "/llm_tools/get_nearest_waypoint")
    return node, state_cli, nearest_cli


@st.cache_resource
def llm_client():
    if not os.getenv("GOOGLE_API_KEY"):
        raise RuntimeError("GOOGLE_API_KEY ist nicht gesetzt.")
    return ChatGoogleGenerativeAI(model=model_name())


def call_trigger(node: Node, client, timeout=2.0):
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    fut = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if fut.result() is None:
        return None
    res = fut.result()
    if not res.success:
        return None
    return res.message


def classify_intent(user_text: str) -> str:
    """
    Deterministisches Routing:
    - 'state'   -> Zustand + Interpretation
    - 'wp'      -> nur nächster Waypoint
    - 'both'    -> beides
    - 'blocked' -> alles andere (off-topic) wird geblockt
    """
    t = user_text.strip().lower()

    # Keywords für Waypoint-Fragen
    wp_kw = [
        "näch", "naech", "nearest", "waypoint", "punkt", "pose", "ziel", "goal",
        "home", "pose a", "pose b"
    ]
    # Keywords für Zustand-Fragen
    state_kw = [
        "zustand", "status", "sensor", "sensordaten", "odom", "laser", "scan",
        "bewegt", "bewegung", "geschwindigkeit", "hindernis", "abstand", "interpret"
    ]

    wp_hit = any(k in t for k in wp_kw)
    state_hit = any(k in t for k in state_kw)

    if wp_hit and state_hit:
        return "both"
    if wp_hit:
        return "wp"
    if state_hit:
        return "state"

    # Wenn nichts passt -> blocken
    return "blocked"


def build_payload(intent: str, user_text: str, rs: dict, nw: dict) -> dict:
    payload = {"question": user_text, "intent": intent}

    if intent in ("state", "both"):
        pose = rs.get("pose") if isinstance(rs, dict) else None
        speed = rs.get("speed_mps") if isinstance(rs, dict) else None
        laser_min = rs.get("laser_min_range_m") if isinstance(rs, dict) else None
        payload["state"] = {
            "pose": pose,
            "speed_mps": speed,
            "laser_min_range_m": laser_min,
        }
        payload["tool_interpretation"] = rs.get("interpretation") if isinstance(rs, dict) else None

    if intent in ("wp", "both"):
        payload["nearest_waypoint"] = nw if isinstance(nw, dict) else None

    return payload


def select_system(intent: str) -> str:
    if intent == "state":
        return SYSTEM_STATE_ONLY
    if intent == "wp":
        return SYSTEM_WP_ONLY
    return SYSTEM_BOTH


def blocked_answer() -> str:
    # wissenschaftlich clean, kurz, ohne Meta
    return (
        "Diese Anwendung beantwortet ausschliesslich Fragen zum:\n"
        "- Roboterzustand (Sensor-/Odometrie-/Laser-Daten) und/oder\n"
        "- naechsten Waypoint (von 4 definierten Punkten).\n"
    )


def fallback_answer(intent: str, rs: dict, nw: dict) -> str:
    lines = []

    if intent in ("state", "both"):
        lines.append("Zustand:")
        pose = (rs or {}).get("pose") or {}
        if isinstance(pose, dict):
            x = pose.get("x")
            y = pose.get("y")
            yaw = pose.get("yaw")
            if x is not None and y is not None:
                lines.append(f"- Position: ({x:.2f}, {y:.2f})")
            if yaw is not None:
                lines.append(f"- Orientierung (yaw): {yaw:.2f} rad")

        speed = (rs or {}).get("speed_mps")
        if speed is not None:
            lines.append(f"- Geschwindigkeit: {float(speed):.2f} m/s")

        laser_min = (rs or {}).get("laser_min_range_m")
        if laser_min is not None:
            lines.append(f"- Minimale Hindernisdistanz: {float(laser_min):.2f} m")

        if len(lines) == 1:
            lines.append("- (keine Daten)")

        lines.append("\nInterpretation:")
        interp = (rs or {}).get("interpretation")
        if interp:
            lines.append(f"- {interp}")
        else:
            lines.append("- Keine Interpretation verfuegbar.")

    if intent in ("wp", "both"):
        if intent != "both":
            lines.append("Naechster Waypoint:")
        else:
            lines.append("\nNaechster Waypoint:")

        if nw and isinstance(nw, dict):
            name = nw.get("name")
            dist = nw.get("distance_m")
            if name is not None and dist is not None:
                try:
                    lines.append(f"- {name} ({float(dist):.2f} m)")
                except Exception:
                    lines.append(f"- {name} ({dist} m)")
            else:
                lines.append("- (nicht bestimmbar)")
        else:
            lines.append("- (nicht bestimmbar)")

    return "\n".join(lines)


def main():
    st.set_page_config(page_title="ROS2 LLM Chatbot (neu)", layout="centered")
    st.title("ROS2 LLM Chatbot (neu)")

    with st.sidebar:
        st.markdown("**Konfiguration**")
        st.markdown(f"- Model: `{model_name()}`")
        st.markdown("- Modus: Intent-Routing + Off-topic Block")

    node, state_cli, nearest_cli = ros_clients()
    llm = llm_client()

    if "history" not in st.session_state:
        st.session_state.history = []

    for msg in st.session_state.history:
        with st.chat_message(msg["role"]):
            st.markdown(msg["text"])

    user_text = st.chat_input("Frage stellen (z.B. 'Wie ist der Zustand des Roboters?')")

    if user_text:
        st.session_state.history.append({"role": "user", "text": user_text})
        with st.chat_message("user"):
            st.markdown(user_text)

        intent = classify_intent(user_text)

        # Off-topic block: KEIN Tool call, KEIN LLM call
        if intent == "blocked":
            answer = blocked_answer()
            with st.chat_message("assistant"):
                st.markdown(answer)
            st.session_state.history.append({"role": "assistant", "text": answer})
            return

        # Tools nur holen, wenn gebraucht
        rs = {}
        nw = {}

        if intent in ("state", "both"):
            state_msg = call_trigger(node, state_cli)
            rs = json.loads(state_msg) if state_msg else {}

        if intent in ("wp", "both"):
            nearest_msg = call_trigger(node, nearest_cli)
            nw = json.loads(nearest_msg) if nearest_msg else {}

        payload = build_payload(intent, user_text, rs, nw)
        prompt = select_system(intent) + "\n\nDaten:\n" + json.dumps(payload, ensure_ascii=False, indent=2)

        with st.chat_message("assistant"):
            try:
                with st.spinner("Antwort wird erstellt..."):
                    out = llm.invoke(prompt)
                    answer = out.content
            except Exception:
                answer = fallback_answer(intent, rs, nw)

            st.markdown(answer)

        st.session_state.history.append({"role": "assistant", "text": answer})


if __name__ == "__main__":
    main()
