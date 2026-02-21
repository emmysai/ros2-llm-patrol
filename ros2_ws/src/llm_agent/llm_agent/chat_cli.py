#!/usr/bin/env python3
import json
import os

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


SYSTEM = (
    "Du bist ein ROS2 Robot Assistant.\n"
    "Du bekommst JSON Sensordaten aus ROS Tools und sollst:\n"
    "1) Zustand kurz zusammenfassen\n"
    "2) interpretieren (z.B. Hindernisnähe, Bewegung, Batterie)\n"
    "3) sagen welcher der 4 Waypoints am nächsten ist (Name + Distanz)\n"
    "Antworte knapp in Bulletpoints."
)

def build_gemini():
    # LangChain (wie in chat2robot üblich) – falls installiert
    from langchain_google_genai import ChatGoogleGenerativeAI
    return ChatGoogleGenerativeAI(model="gemini-2.5-flash")


class ChatCLI(Node):
    def __init__(self):
        super().__init__("llm_chat_cli")
        self.state_cli = self.create_client(Trigger, "/llm_tools/get_robot_state")
        self.nearest_cli = self.create_client(Trigger, "/llm_tools/get_nearest_waypoint")

    def call_trigger(self, client, timeout=3.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None, f"Service {client.srv_name} not available"
        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.result() is None:
            return None, f"Timeout calling {client.srv_name}"
        res = fut.result()
        if not res.success:
            return None, f"{client.srv_name} returned success=False"
        return res.message, None


def main():
    if not os.getenv("GOOGLE_API_KEY"):
        print("ERROR: GOOGLE_API_KEY not set in environment.")
        return

    llm = build_gemini()

    rclpy.init()
    node = ChatCLI()

    print("Gemini Chatbot gestartet. 'exit' zum Beenden.")
    while True:
        user = input("\nYou> ").strip()
        if user.lower() in ("exit", "quit"):
            break

        state_msg, e1 = node.call_trigger(node.state_cli)
        near_msg, e2 = node.call_trigger(node.nearest_cli)

        payload = {
            "user_message": user,
            "robot_state": None if state_msg is None else json.loads(state_msg),
            "nearest_waypoint": None if near_msg is None else json.loads(near_msg),
            "errors": [x for x in [e1, e2] if x],
        }

        prompt = SYSTEM + "\n\nJSON:\n" + json.dumps(payload, ensure_ascii=False, indent=2)
        out = llm.invoke(prompt)
        print("\nAssistant:\n" + out.content)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
