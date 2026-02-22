#!/usr/bin/env python3
import json
import os

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from langchain_core.messages import HumanMessage, AIMessage
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.tools import tool
from langchain.agents import AgentExecutor, create_tool_calling_agent
from langchain_google_genai import ChatGoogleGenerativeAI


SYSTEM_PROMPT = (
    "Du bist ein ROS2 Robot Assistant als Tool-using Agent.\n"
    "WICHTIGES VORGEHEN (immer):\n"
    "1) Rufe zuerst das Tool get_robot_state auf.\n"
    "2) Rufe danach das Tool get_nearest_waypoint auf.\n"
    "3) Dann schreibe die Antwort.\n\n"
    "AUFGABE:\n"
    "- Fasse den Roboterzustand kurz zusammen.\n"
    "- Interpretiere ihn (Bewegung, Hindernisnähe, Stabilität anhand IMU).\n"
    "- Gib an, welcher der 4 Waypoints aktuell am nächsten ist (Name + Distanz).\n"
    "Antworte kurz in Bulletpoints.\n"
)

FULL_PROMPT = ChatPromptTemplate.from_messages([
    ("system", SYSTEM_PROMPT),
    ("placeholder", "{chat_history}"),
    ("human", "{input}"),
    ("placeholder", "{agent_scratchpad}"),
])


class RosToolClient(Node):
    def __init__(self):
        super().__init__("llm_chat_cli_agentic")
        self.state_cli = self.create_client(Trigger, "/llm_tools/get_robot_state")
        self.nearest_cli = self.create_client(Trigger, "/llm_tools/get_nearest_waypoint")

    def call_trigger_json(self, client, timeout=3.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return {"error": f"Service {client.srv_name} not available"}
        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.result() is None:
            return {"error": f"Timeout calling {client.srv_name}"}
        res = fut.result()
        if not res.success:
            return {"error": f"{client.srv_name} returned success=False"}
        try:
            return json.loads(res.message)
        except Exception:
            return {"error": "Invalid JSON returned", "raw": res.message}


def main():
    if not os.getenv("GOOGLE_API_KEY"):
        print("ERROR: GOOGLE_API_KEY not set in environment.")
        return

    rclpy.init()
    node = RosToolClient()

    # LLM (Gemini)
    llm = ChatGoogleGenerativeAI(model="gemini-2.5-flash", temperature=0.3)

    # Agent-Tools (LLM entscheidet selbst, wann sie aufgerufen werden)
    @tool("get_robot_state")
    def get_robot_state() -> str:
        """Gibt JSON mit internen Sensorwerten zurück (pose, odom-speed, laser stats, imu, interpretation)."""
        data = node.call_trigger_json(node.state_cli)
        return json.dumps(data, ensure_ascii=False)

    @tool("get_nearest_waypoint")
    def get_nearest_waypoint() -> str:
        """Gibt JSON zum nächsten der 4 Waypoints zurück (Name, Distanz, Koordinaten, Pose)."""
        data = node.call_trigger_json(node.nearest_cli)
        return json.dumps(data, ensure_ascii=False)

    tools = [get_robot_state, get_nearest_waypoint]

    agent = create_tool_calling_agent(llm, tools, FULL_PROMPT)
    executor = AgentExecutor(
        agent=agent,
        tools=tools,
        verbose=True,  # zeigt Toolcalls im Terminal -> Beweis für Agentic Tool Calling
        handle_parsing_errors=True
    )

    chat_history = []

    print("Agentic Tool-Calling Chat gestartet. 'exit' zum Beenden.")
    while True:
        user = input("\nYou> ").strip()
        if user.lower() in ("exit", "quit"):
            break

        chat_history.append(HumanMessage(content=user))

        result = executor.invoke({
            "input": user,
            "chat_history": chat_history
        })

        output = result["output"]
        print("\nAssistant:\n" + output)

        chat_history.append(AIMessage(content=output))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()