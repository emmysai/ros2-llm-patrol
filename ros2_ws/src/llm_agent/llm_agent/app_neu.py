#!/usr/bin/env python3
import json
import os

import streamlit as st

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from langchain_core.messages import HumanMessage, AIMessage
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.tools import tool

# AgentExecutor import (kompatibel)
try:
    from langchain.agents import AgentExecutor
except Exception:
    from langchain.agents.agent import AgentExecutor

from langchain.agents import create_tool_calling_agent
from langchain_google_genai import ChatGoogleGenerativeAI


SYSTEM_PROMPT = (
    "Du bist ein ROS2 LLM-Agent für einen mobilen Roboter.\n"
    "Du darfst NUR Fragen zu diesen Themen beantworten:\n"
    "A) Robot-Zustand: Sensorwerte zusammenfassen und interpretieren (odom/scan/imu/pose)\n"
    "B) Nächster Punkt: welcher der 4 Waypoints am nächsten ist (Name + Distanz)\n"
    "C) Kombiniert: Zustand + nächster Punkt\n\n"
    "WICHTIG:\n"
    "- Wenn die Nutzerfrage NICHT zu A/B/C gehört, antworte IMMER exakt:\n"
    "  \"Ich kann nur Fragen zum Roboterzustand und zum nächsten der 4 Waypoints beantworten. "
    "Frag z.B.: 'Wie ist der Zustand?' oder 'Welcher Punkt ist am nächsten?'\"\n"
    "- In diesem Fall: KEINE Tools aufrufen.\n\n"
    "TOOL-REGEL:\n"
    "- Wenn die Nutzerfrage zu A/B/C gehört: rufe zuerst get_robot_state auf und danach get_nearest_waypoint.\n"
    "- Danach antworte kurz in Bulletpoints.\n"
    "- Interpretiere: Bewegung (Speed), Hindernisnähe (Laser), Stabilität/Rotation (IMU).\n"
)

FULL_PROMPT = ChatPromptTemplate.from_messages([
    ("system", SYSTEM_PROMPT),
    ("placeholder", "{chat_history}"),
    ("human", "{input}"),
    ("placeholder", "{agent_scratchpad}"),
])


def model_name() -> str:
    return os.getenv("GEMINI_MODEL", "gemini-2.5-flash")


@st.cache_resource
def ros_node_and_clients():
    rclpy.init(args=None)
    node = Node("llm_streamlit_agentic_ui")
    state_cli = node.create_client(Trigger, "/llm_tools/get_robot_state")
    nearest_cli = node.create_client(Trigger, "/llm_tools/get_nearest_waypoint")
    return node, state_cli, nearest_cli


def call_trigger_json(node: Node, client, timeout=3.0):
    if not client.wait_for_service(timeout_sec=timeout):
        return {"error": f"Service {client.srv_name} not available"}
    fut = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if fut.result() is None:
        return {"error": f"Timeout calling {client.srv_name}"}
    res = fut.result()
    if not res.success:
        return {"error": f"{client.srv_name} returned success=False"}
    try:
        return json.loads(res.message)
    except Exception:
        return {"error": "Invalid JSON returned", "raw": res.message}


@st.cache_resource
def agent_executor():
    if not os.getenv("GOOGLE_API_KEY"):
        raise RuntimeError("GOOGLE_API_KEY ist nicht gesetzt.")

    node, state_cli, nearest_cli = ros_node_and_clients()

    llm = ChatGoogleGenerativeAI(model=model_name(), temperature=0.3)

    @tool("get_robot_state")
    def get_robot_state() -> str:
        """Gibt JSON mit internen Sensorwerten zurück (odom/scan/imu/pose + interpretation)."""
        data = call_trigger_json(node, state_cli)
        return json.dumps(data, ensure_ascii=False)

    @tool("get_nearest_waypoint")
    def get_nearest_waypoint() -> str:
        """Gibt JSON zum nächsten der 4 Waypoints zurück (Name + Distanz)."""
        data = call_trigger_json(node, nearest_cli)
        return json.dumps(data, ensure_ascii=False)

    tools = [get_robot_state, get_nearest_waypoint]

    agent = create_tool_calling_agent(llm, tools, FULL_PROMPT)
    executor = AgentExecutor(
        agent=agent,
        tools=tools,
        verbose=False,  # UI sauber; für Debug -> True
        handle_parsing_errors=True,
    )
    return executor


def off_topic_message() -> str:
    return ("Ich kann nur Fragen zum Roboterzustand und zum nächsten der 4 Waypoints beantworten. "
            "Frag z.B.: 'Wie ist der Zustand?' oder 'Welcher Punkt ist am nächsten?'")


def is_allowed_topic(user_text: str) -> bool:
    u = user_text.lower()
    return any(k in u for k in [
        "zustand", "status", "sensor", "sensordaten", "robot", "roboter",
        "imu", "odom", "scan", "laserscan", "pose", "position", "geschwindigkeit",
        "nearest", "nächster", "naechster", "punkt", "waypoint", "ziel", "distanz", "distance"
    ])


def main():
    st.set_page_config(page_title="ROS2 LLM Chatbot (Agentic UI)", layout="centered")
    st.title("ROS2 LLM Chatbot")

    with st.sidebar:
        st.markdown("**Konfiguration**")
        st.markdown(f"- Model: `{model_name()}`")
        st.markdown("- Topic-Gate: nur Zustand / nächster Waypoint")
        if st.button("Chat zurücksetzen"):
            st.session_state.history = []
            st.session_state.lc_history = []
            st.rerun()

    executor = agent_executor()

    if "history" not in st.session_state:
        st.session_state.history = []  # für Anzeige
    if "lc_history" not in st.session_state:
        st.session_state.lc_history = []  # LangChain Messages

    # History rendern
    for msg in st.session_state.history:
        with st.chat_message(msg["role"]):
            st.markdown(msg["text"])

    user_text = st.chat_input("Frage stellen (z.B. 'Wie ist der Zustand?' / 'Welcher Punkt ist am nächsten?')")

    if user_text:
        # user anzeigen
        st.session_state.history.append({"role": "user", "text": user_text})
        with st.chat_message("user"):
            st.markdown(user_text)

        st.session_state.lc_history.append(HumanMessage(content=user_text))

        # Topic Gate (wie im CLI)
        if not is_allowed_topic(user_text):
            msg = off_topic_message()
            with st.chat_message("assistant"):
                st.markdown(msg)
            st.session_state.history.append({"role": "assistant", "text": msg})
            st.session_state.lc_history.append(AIMessage(content=msg))
            return

        # Agentic Tool Calling
        with st.chat_message("assistant"):
            with st.spinner("Antwort wird erstellt..."):
                result = executor.invoke({
                    "input": user_text,
                    "chat_history": st.session_state.lc_history
                })
                answer = result["output"]
                st.markdown(answer)

        st.session_state.history.append({"role": "assistant", "text": answer})
        st.session_state.lc_history.append(AIMessage(content=answer))


if __name__ == "__main__":
    main()