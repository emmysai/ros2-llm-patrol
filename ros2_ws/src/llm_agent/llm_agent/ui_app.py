#!/usr/bin/env python3
import os
import json
import time

import streamlit as st

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from langchain_google_genai import ChatGoogleGenerativeAI


SYSTEM = """
Du bist ein wissenschaftlicher ROS2 Robot Assistant.

Du bekommst reale Sensordaten aus einem TurtleBot3 in Gazebo.

Verfuegbare Daten (nur falls vorhanden):
- Position (x,y)
- Orientierung (yaw)
- Lineare Geschwindigkeit (m/s)
- Winkelgeschwindigkeit (rad/s)
- Minimale Hindernisdistanz (LaserScan)
- Naechster Waypoint (Name + Distanz)

Deine Aufgabe:
1) Zustand strukturiert zusammenfassen
2) Kurz interpretieren (Bewegung, Hindernisnaehe)
3) Naechsten Waypoint angeben

Regeln:
- Keine Einleitung
- Keine Meta-Kommentare
- Keine Erwaehnung von Tools/JSON/Debug
- Keine Annahmen ueber nicht vorhandene Sensoren (z.B. Batterie)
- Nutze exakt dieses Ausgabeformat:

Zustand:
- ...
- ...

Interpretation:
- ...
- ...

Naechster Waypoint:
- ...
"""


def model_name() -> str:
    return os.getenv("GEMINI_MODEL", "gemini-2.5-flash")


@st.cache_resource
def ros_clients():
    # ROS init einmal pro Streamlit Prozess
    rclpy.init(args=None)
    node = Node("llm_streamlit_ui")

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


def extract_min_laser(scan_obj):
    # scan_obj expected from your tool JSON; if not present -> None
    # We keep it robust: handle "laser_min_range_m" from tool if provided.
    if not isinstance(scan_obj, dict):
        return None
    v = scan_obj.get("laser_min_range_m")
    return v


def build_clean_payload(user_text: str, rs: dict, nw: dict):
    """
    Wir geben dem LLM NUR das, was er braucht (wissenschaftlich clean).
    """
    payload = {"question": user_text}

    pose = rs.get("pose") if isinstance(rs, dict) else None
    speed = rs.get("speed_mps") if isinstance(rs, dict) else None
    laser_min = rs.get("laser_min_range_m") if isinstance(rs, dict) else None

    payload["state"] = {
        "pose": pose,
        "speed_mps": speed,
        "laser_min_range_m": laser_min,
    }
    payload["nearest_waypoint"] = nw if isinstance(nw, dict) else None
    return payload


def fallback_answer(rs: dict, nw: dict):
    """
    Falls Gemini nicht erreichbar ist, liefern wir trotzdem die 3 Blöcke.
    """
    lines = []

    # Zustand
    lines.append("Zustand:")
    pose = (rs or {}).get("pose")
    speed = (rs or {}).get("speed_mps")
    laser_min = (rs or {}).get("laser_min_range_m")

    if pose and isinstance(pose, dict):
        x = pose.get("x")
        y = pose.get("y")
        yaw = pose.get("yaw")
        if x is not None and y is not None:
            lines.append(f"- Position: ({x:.2f}, {y:.2f})")
        if yaw is not None:
            lines.append(f"- Orientierung (yaw): {yaw:.2f} rad")
    if speed is not None:
        lines.append(f"- Geschwindigkeit: {float(speed):.2f} m/s")
    if laser_min is not None:
        lines.append(f"- Minimale Hindernisdistanz: {float(laser_min):.2f} m")

    if len(lines) == 1:
        lines.append("- (keine Daten)")

    # Interpretation
    lines.append("\nInterpretation:")
    interp = (rs or {}).get("interpretation")
    if interp:
        # Interpretation aus deinem Tool-Node
        # (Falls du dort schon eine wissenschaftliche Interpretation baust)
        lines.append(f"- {interp}")
    else:
        lines.append("- Keine Interpretation verfuegbar.")

    # Nächster Waypoint
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
    st.set_page_config(page_title="ROS2 LLM Chatbot", layout="centered")
    st.title("ROS2 LLM Chatbot")

    # Minimalistische Sidebar
    with st.sidebar:
        st.markdown("**Konfiguration**")
        st.markdown(f"- Model: `{model_name()}`")

    # init clients
    node, state_cli, nearest_cli = ros_clients()
    llm = llm_client()

    if "history" not in st.session_state:
        st.session_state.history = []

    # display chat history
    for msg in st.session_state.history:
        with st.chat_message(msg["role"]):
            st.markdown(msg["text"])

    user_text = st.chat_input("Frage stellen (z.B. 'Wie ist der Zustand des Roboters?')")

    if user_text:
        # show user
        st.session_state.history.append({"role": "user", "text": user_text})
        with st.chat_message("user"):
            st.markdown(user_text)

        # call tools
        state_msg = call_trigger(node, state_cli)
        nearest_msg = call_trigger(node, nearest_cli)

        rs = json.loads(state_msg) if state_msg else {}
        nw = json.loads(nearest_msg) if nearest_msg else {}

        # build clean payload (only required data)
        payload = build_clean_payload(user_text, rs, nw)
        prompt = SYSTEM + "\n\nDaten:\n" + json.dumps(payload, ensure_ascii=False, indent=2)

        # LLM (clean output only)
        with st.chat_message("assistant"):
            try:
                with st.spinner("Antwort wird erstellt..."):
                    out = llm.invoke(prompt)
                    answer = out.content
            except Exception:
                answer = fallback_answer(rs, nw)

            st.markdown(answer)

        st.session_state.history.append({"role": "assistant", "text": answer})


if __name__ == "__main__":
    main()
