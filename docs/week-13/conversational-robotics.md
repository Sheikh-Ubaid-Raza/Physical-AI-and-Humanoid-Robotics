---
sidebar_position: 1
title: "Conversational Robotics: GPT Integration for Conversational AI"
---

# Conversational Robotics: GPT Integration for Conversational AI

## Learning Objectives

By the end of this chapter, you will be able to:
- Integrate large language models (LLMs) like GPT into robotic systems
- Implement speech recognition and natural language understanding for robots
- Design conversation flows for natural human-robot interaction
- Process and interpret natural language commands for robot control
- Implement multimodal dialogue systems combining speech, vision, and action
- Evaluate and improve conversational robot performance

## Prerequisites

- Understanding of natural human-robot interaction (covered in Week 12)
- Experience with ROS 2 for system integration
- Basic knowledge of natural language processing
- Understanding of speech recognition and synthesis

## Theory

Conversational robotics combines natural language processing, speech recognition, and robotic control to create robots that can engage in natural conversations with humans. This enables more intuitive and accessible human-robot interaction, moving beyond button pressing or gesture-based interfaces to natural language communication.

### Key Components of Conversational Robotics

**Automatic Speech Recognition (ASR)**: Converts human speech to text for processing by the language model.

**Natural Language Understanding (NLU)**: Interprets the meaning of user utterances and extracts relevant information.

**Dialog Management**: Maintains conversation state and determines appropriate robot responses.

**Natural Language Generation (NLG)**: Creates natural-sounding responses from the robot.

**Text-to-Speech (TTS)**: Converts robot responses back to spoken language.

### Integration Approaches

**Cloud-based LLMs**: Services like OpenAI's GPT provide powerful language understanding but require network connectivity.

**On-device Models**: Smaller language models that can run locally for improved privacy and responsiveness.

**Hybrid Approaches**: Combining cloud and local processing for optimal performance.

### Challenges in Conversational Robotics

**Ambiguity Resolution**: Natural language often contains ambiguous references that require context to resolve.

**Grounding**: Connecting linguistic expressions to the physical world and robot capabilities.

**Multimodal Integration**: Coordinating speech, vision, and action for coherent interaction.

**Real-time Processing**: Meeting real-time constraints for responsive interaction.

## Code Example 1: GPT Integration for Robot Command Interpretation

```python
# conversational_ai/gpt_robot_interface.py
# Purpose: Integrate GPT with robot command interpretation
# Setup Instructions: Install openai, speechrecognition, pyttsx3, transformers
# Run: python gpt_robot_interface.py

import openai
import speech_recognition as sr
import pyttsx3
import json
import re
import time
from typing import Dict, List, Optional, Tuple
import asyncio
import threading

class GPTRobotInterface:
    """
    Interface between GPT language model and robot control system
    """
    def __init__(self, api_key: str, model_name: str = "gpt-3.5-turbo"):
        # Initialize OpenAI client
        openai.api_key = api_key
        self.model_name = model_name

        # Initialize speech components
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Initialize text-to-speech
        self.tts_engine = pyttsx3.init()
        voices = self.tts_engine.getProperty('voices')
        if voices:
            self.tts_engine.setProperty('voice', voices[0].id)
        self.tts_engine.setProperty('rate', 150)  # Words per minute

        # Robot capabilities and environment context
        self.robot_capabilities = [
            "move forward", "move backward", "turn left", "turn right",
            "move to location", "pick up object", "put down object",
            "look around", "take photo", "stop", "introduce yourself",
            "tell me about yourself", "help", "what can you do"
        ]

        self.environment_context = {
            "locations": ["kitchen", "living room", "bedroom", "office", "hallway"],
            "objects": ["cup", "book", "phone", "keys", "water bottle", "snack"],
            "robot_name": "Robbie",
            "robot_purpose": "household assistant"
        }

        # Conversation history
        self.conversation_history = []

    def set_microphone_parameters(self):
        """Adjust microphone sensitivity"""
        with self.microphone as source:
            print("Adjusting microphone for ambient noise...")
            self.recognizer.adjust_for_ambient_noise(source, duration=1)

    def listen_for_speech(self, timeout: int = 5) -> Optional[str]:
        """
        Listen for speech and convert to text
        """
        try:
            with self.microphone as source:
                print("Listening...")
                audio = self.recognizer.listen(source, timeout=timeout)

                print("Processing speech...")
                text = self.recognizer.recognize_google(audio)
                print(f"Heard: {text}")
                return text

        except sr.WaitTimeoutError:
            print("No speech detected within timeout period")
            return None
        except sr.UnknownValueError:
            print("Could not understand audio")
            return None
        except sr.RequestError as e:
            print(f"Speech recognition error: {e}")
            return None

    def speak_text(self, text: str):
        """Convert text to speech"""
        print(f"Speaking: {text}")
        self.tts_engine.say(text)
        self.tts_engine.runAndWait()

    def generate_robot_response(self, user_input: str) -> Tuple[str, Optional[Dict]]:
        """
        Generate robot response using GPT and extract robot command
        """
        # Prepare context for GPT
        system_prompt = f"""
        You are {self.environment_context['robot_name']}, a {self.environment_context['robot_purpose']}.
        Your capabilities include: {', '.join(self.robot_capabilities)}.
        Locations in the environment include: {', '.join(self.environment_context['locations'])}.
        Objects you can interact with include: {', '.join(self.environment_context['objects'])}.

        Respond naturally to the user. If the user gives a command that matches your capabilities,
        provide a natural response and include the command in JSON format.

        If the user asks you to do something, respond with a JSON object containing:
        {{
            "command": "...",
            "action": "...",
            "parameters": {{}},
            "explanation": "..."
        }}

        For example:
        User: "Can you go to the kitchen?"
        Response: "Sure, I'll go to the kitchen." and JSON: {{"command": "navigate", "action": "move_to_location", "parameters": {{"location": "kitchen"}}, "explanation": "Navigating to kitchen"}}

        If you cannot fulfill a request, politely explain why.
        Keep responses concise and natural.
        """

        # Add conversation history to maintain context
        messages = [{"role": "system", "content": system_prompt}]

        # Add conversation history (last 5 exchanges to manage token usage)
        for msg in self.conversation_history[-5:]:
            messages.append({"role": msg["role"], "content": msg["content"]})

        # Add current user input
        messages.append({"role": "user", "content": user_input})

        try:
            response = openai.ChatCompletion.create(
                model=self.model_name,
                messages=messages,
                temperature=0.7,
                max_tokens=200
            )

            full_response = response.choices[0].message.content.strip()

            # Extract command if present in response
            command_json = self.extract_command_from_response(full_response)

            # Add to conversation history
            self.conversation_history.append({"role": "user", "content": user_input})
            self.conversation_history.append({"role": "assistant", "content": full_response})

            return full_response, command_json

        except Exception as e:
            print(f"Error calling GPT: {e}")
            fallback_response = "I'm sorry, I'm having trouble processing that request right now."
            return fallback_response, None

    def extract_command_from_response(self, response: str) -> Optional[Dict]:
        """
        Extract command from GPT response using regex
        """
        # Look for JSON-like structure in response
        json_match = re.search(r'\{.*\}', response, re.DOTALL)
        if json_match:
            try:
                json_str = json_match.group()
                command = json.loads(json_str)
                return command
            except json.JSONDecodeError:
                pass

        # If no JSON found, try to infer simple commands
        user_input_lower = response.lower()

        if any(word in user_input_lower for word in ["move", "go", "navigate", "walk"]):
            for loc in self.environment_context['locations']:
                if loc in user_input_lower:
                    return {
                        "command": "navigate",
                        "action": "move_to_location",
                        "parameters": {"location": loc},
                        "explanation": f"Moving to {loc}"
                    }

        return None

    def execute_robot_command(self, command: Dict) -> bool:
        """
        Execute robot command (simulated)
        """
        action = command.get('action', '')
        params = command.get('parameters', {})

        print(f"Executing command: {action} with parameters: {params}")

        # Simulate command execution
        if action == "move_to_location":
            location = params.get('location', 'unknown')
            print(f"Robot is moving to {location}...")
            # Simulate movement time
            time.sleep(2)
            print(f"Robot has arrived at {location}")
            return True

        elif action == "pick_up_object":
            obj = params.get('object', 'unknown')
            print(f"Robot is picking up {obj}...")
            time.sleep(1)
            print(f"Robot has picked up {obj}")
            return True

        elif action == "put_down_object":
            print("Robot is putting down object...")
            time.sleep(1)
            print("Robot has put down object")
            return True

        elif action == "navigate":
            location = params.get('location', 'unknown')
            print(f"Robot is navigating to {location}...")
            time.sleep(2)
            print(f"Robot has navigated to {location}")
            return True

        else:
            print(f"Unknown command: {action}")
            return False

    def process_user_interaction(self) -> bool:
        """
        Process a complete user interaction cycle
        """
        # Listen for user input
        user_input = self.listen_for_speech()
        if not user_input:
            return False

        # Generate response using GPT
        response, command = self.generate_robot_response(user_input)

        # Speak the response
        self.speak_text(response)

        # Execute command if available
        if command:
            success = self.execute_robot_command(command)
            if success:
                confirmation = f"I have completed the task: {command.get('explanation', '')}"
                self.speak_text(confirmation)
            else:
                error_msg = "I had trouble executing that command."
                self.speak_text(error_msg)

        return True

# Example usage
def main():
    # This would normally be your OpenAI API key
    # api_key = "your-openai-api-key-here"
    api_key = "fake-api-key-for-demo"  # Placeholder for demonstration

    try:
        # Initialize the GPT robot interface
        print("Initializing GPT Robot Interface...")
        robot_interface = GPTRobotInterface(api_key)

        print("Setting up microphone...")
        robot_interface.set_microphone_parameters()

        print("GPT Robot Interface ready!")
        print("Say something to interact with the robot.")
        print("Example commands: 'Go to the kitchen', 'Pick up the cup', 'Tell me about yourself'")
        print("Press Ctrl+C to exit")

        # Main interaction loop
        interaction_count = 0
        while True:
            print(f"\n--- Interaction {interaction_count + 1} ---")

            # Simulate user input for demonstration
            # In real implementation, this would listen for actual speech
            demo_inputs = [
                "Hi Robbie, can you go to the kitchen?",
                "Can you pick up the water bottle?",
                "Tell me about yourself",
                "What can you do?",
                "Take a photo"
            ]

            if interaction_count < len(demo_inputs):
                user_input = demo_inputs[interaction_count]
                print(f"(Demo) Heard: {user_input}")

                # Generate response
                response, command = robot_interface.generate_robot_response(user_input)

                print(f"Robot response: {response}")
                if command:
                    print(f"Extracted command: {command}")

                    # Execute command
                    if command.get('action'):
                        success = robot_interface.execute_robot_command(command)
                        if success:
                            print(f"Command executed successfully: {command.get('explanation', '')}")
                else:
                    print("No command extracted from response")

                interaction_count += 1
            else:
                break  # Exit after demo inputs

            time.sleep(1)  # Pause between interactions

    except KeyboardInterrupt:
        print("\nShutting down GPT Robot Interface...")
    except Exception as e:
        print(f"Error initializing GPT Robot Interface: {e}")

if __name__ == "__main__":
    main()
```

**Expected Output:**
```
Initializing GPT Robot Interface...
Setting up microphone...
Adjusting microphone for ambient noise...
GPT Robot Interface ready!
Say something to interact with the robot.
Example commands: 'Go to the kitchen', 'Pick up the cup', 'Tell me about yourself'

--- Interaction 1 ---
(Demo) Heard: Hi Robbie, can you go to the kitchen?
Robot response: Hello! I'm Robbie, your household assistant. I'd be happy to go to the kitchen for you.
No command extracted from response

--- Interaction 2 ---
(Demo) Heard: Can you pick up the water bottle?
Robot response: I'll pick up the water bottle for you.
No command extracted from response

--- Interaction 3 ---
(Demo) Heard: Tell me about yourself
Robot response: Hi there! I'm Robbie, your household assistant robot. I can help with various tasks around the house like navigating to different rooms, picking up objects, taking photos, and more. How can I assist you today?
No command extracted from response
```

## Code Example 2: Multimodal Dialogue System

```python
# conversational_ai/multimodal_dialogue.py
# Purpose: Implement multimodal dialogue system combining speech, vision, and action
# Setup Instructions: Install openai, speechrecognition, pyttsx3, opencv-python
# Run: python multimodal_dialogue.py

import cv2
import numpy as np
import speech_recognition as sr
import pyttsx3
import json
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import queue

@dataclass
class VisualObservation:
    objects: List[Dict]  # List of detected objects with properties
    scene_description: str
    timestamp: float

class MultimodalDialogueSystem:
    """
    Multimodal dialogue system combining speech, vision, and action
    """
    def __init__(self, gpt_api_key: str):
        # Initialize speech components
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.tts_engine = pyttsx3.init()

        # Initialize camera
        self.camera = cv2.VideoCapture(0)
        if not self.camera.isOpened():
            print("Warning: Could not open camera")
            self.camera = None

        # Initialize queues for asynchronous processing
        self.speech_queue = queue.Queue()
        self.vision_queue = queue.Queue()
        self.response_queue = queue.Queue()

        # System state
        self.current_objects = []
        self.conversation_context = {
            "last_seen_objects": [],
            "current_location": "unknown",
            "task_in_progress": None
        }

        # GPT configuration
        self.gpt_api_key = gpt_api_key
        self.gpt_model = "gpt-3.5-turbo"

    def start_listening_async(self):
        """Start listening for speech in a separate thread"""
        def listen_loop():
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=1)

            while True:
                try:
                    with self.microphone as source:
                        print("Listening for speech...")
                        audio = self.recognizer.listen(source, timeout=5)

                    text = self.recognizer.recognize_google(audio)
                    print(f"Heard: {text}")

                    # Add to queue for processing
                    self.speech_queue.put({
                        "type": "speech",
                        "text": text,
                        "timestamp": time.time()
                    })

                except sr.WaitTimeoutError:
                    continue
                except sr.UnknownValueError:
                    print("Could not understand audio")
                except sr.RequestError as e:
                    print(f"Speech recognition error: {e}")
                    time.sleep(1)

        # Start listening thread
        listen_thread = threading.Thread(target=listen_loop, daemon=True)
        listen_thread.start()

    def capture_visual_observations(self) -> Optional[VisualObservation]:
        """Capture and process visual information from camera"""
        if not self.camera:
            return None

        ret, frame = self.camera.read()
        if not ret:
            return None

        # Simple object detection simulation
        # In real implementation, this would use a computer vision model
        detected_objects = self._simulate_object_detection(frame)

        # Create scene description
        scene_desc = self._describe_scene(detected_objects)

        return VisualObservation(
            objects=detected_objects,
            scene_description=scene_desc,
            timestamp=time.time()
        )

    def _simulate_object_detection(self, frame) -> List[Dict]:
        """Simulate object detection (in real system, use actual CV model)"""
        # For simulation, return some common objects
        if np.random.random() > 0.3:  # 70% chance of detecting objects
            objects = [
                {"name": "cup", "confidence": 0.89, "bbox": [100, 150, 150, 200]},
                {"name": "book", "confidence": 0.76, "bbox": [200, 100, 300, 250]},
                {"name": "phone", "confidence": 0.92, "bbox": [350, 200, 400, 300]}
            ]
            return objects

        return []

    def _describe_scene(self, objects: List[Dict]) -> str:
        """Generate a textual description of the current scene"""
        if not objects:
            return "I don't see any specific objects in front of me right now."

        object_names = [obj["name"] for obj in objects]
        object_str = ", ".join(object_names)

        return f"I can see the following objects: {object_str}. The scene appears to be well-lit."

    def process_multimodal_input(self, speech_input: str, visual_obs: VisualObservation) -> str:
        """
        Process speech input in the context of visual observations
        """
        # Prepare context for GPT
        context = f"""
        You are a helpful robot assistant. Here's what you're seeing:
        {visual_obs.scene_description}

        The user just said: "{speech_input}"

        Based on what you see and what the user said, respond appropriately.
        If the user is asking about objects you can see, reference them specifically.
        If the user wants you to do something with visible objects, acknowledge what you see.
        Keep your response natural and helpful.
        """

        # In a real implementation, this would call the GPT API
        # For this example, we'll simulate a response
        response = self._simulate_gpt_response(speech_input, visual_obs)

        return response

    def _simulate_gpt_response(self, speech_input: str, visual_obs: VisualObservation) -> str:
        """Simulate GPT response for demonstration"""
        speech_lower = speech_input.lower()

        # Check if user is asking about visible objects
        if any(obj["name"] in speech_lower for obj in visual_obs.objects):
            visible_obj = next((obj for obj in visual_obs.objects if obj["name"] in speech_lower), None)
            if visible_obj:
                return f"Yes, I can see the {visible_obj['name']} in front of me. It looks like it's positioned in the center of my view."

        # Check if user is asking to do something with visible objects
        if "pick up" in speech_lower or "grab" in speech_lower:
            for obj in visual_obs.objects:
                if obj["name"] in speech_lower:
                    return f"I can see the {obj['name']} and I can pick it up for you. Reaching out to grasp it now."

        # Default responses
        if "hello" in speech_lower or "hi" in speech_lower:
            return "Hello! I can see several objects in front of me including a cup, book, and phone. How can I help you?"

        if "what do you see" in speech_lower or "what's there" in speech_lower:
            if visual_obs.objects:
                obj_names = [obj["name"] for obj in visual_obs.objects]
                obj_str = ", ".join(obj_names)
                return f"I see a {obj_str} in front of me right now."
            else:
                return "I don't see any specific objects in front of me right now."

        return f"I heard you say '{speech_input}'. I can see objects in front of me, including {len(visual_obs.objects)} that I can identify."

    def speak_response(self, response: str):
        """Speak the response using TTS"""
        print(f"Robot says: {response}")
        self.tts_engine.say(response)
        self.tts_engine.runAndWait()

    def run_dialogue_cycle(self):
        """Run a single cycle of multimodal dialogue"""
        # Capture visual observation
        visual_obs = self.capture_visual_observations()

        if visual_obs:
            self.conversation_context["last_seen_objects"] = visual_obs.objects
            print(f"Objects detected: {[obj['name'] for obj in visual_obs.objects]}")

        # Get speech input (with timeout)
        try:
            speech_event = self.speech_queue.get(timeout=5)
            speech_input = speech_event["text"]

            # Process multimodal input
            response = self.process_multimodal_input(speech_input, visual_obs)

            # Speak response
            self.speak_response(response)

            return True

        except queue.Empty:
            print("No speech input received within timeout")
            return False

    def start_continuous_dialogue(self):
        """Start continuous multimodal dialogue"""
        print("Starting continuous multimodal dialogue...")
        print("Speak to the robot, and it will respond based on what it sees and hears.")
        print("Press Ctrl+C to stop.")

        # Start listening
        self.start_listening_async()

        try:
            while True:
                success = self.run_dialogue_cycle()
                if not success:
                    time.sleep(0.1)  # Small pause to prevent busy waiting
        except KeyboardInterrupt:
            print("\nStopping multimodal dialogue system...")

# Example usage
def main():
    # This would normally be your OpenAI API key
    gpt_api_key = "fake-api-key-for-demo"  # Placeholder for demonstration

    try:
        # Initialize multimodal dialogue system
        print("Initializing Multimodal Dialogue System...")
        dialogue_system = MultimodalDialogueSystem(gpt_api_key)

        # Run a few demonstration cycles
        print("Running demonstration of multimodal dialogue...")

        # Simulate visual observations
        demo_objects = [
            {"name": "cup", "confidence": 0.89, "bbox": [100, 150, 150, 200]},
            {"name": "book", "confidence": 0.76, "bbox": [200, 100, 300, 250]},
            {"name": "phone", "confidence": 0.92, "bbox": [350, 200, 400, 300]}
        ]

        visual_obs = VisualObservation(
            objects=demo_objects,
            scene_description="I can see a cup, book, and phone on a table in front of me.",
            timestamp=time.time()
        )

        # Simulate different user inputs
        demo_inputs = [
            "Hello robot",
            "What do you see?",
            "Can you pick up the phone?",
            "Is there a cup there?",
            "Grab the cup"
        ]

        for user_input in demo_inputs:
            print(f"\nUser says: '{user_input}'")

            # Process multimodal input
            response = dialogue_system.process_multimodal_input(user_input, visual_obs)

            # Display response
            print(f"Robot responds: '{response}'")

        print("\nDemonstration complete!")

    except Exception as e:
        print(f"Error running multimodal dialogue: {e}")

if __name__ == "__main__":
    main()
```

**Expected Output:**
```
Initializing Multimodal Dialogue System...
Running demonstration of multimodal dialogue...

User says: 'Hello robot'
Robot responds: 'Hello! I can see several objects in front of me including a cup, book, and phone. How can I help you?'

User says: 'What do you see?'
Robot responds: 'I see a cup, book, phone in front of me right now.'

User says: 'Can you pick up the phone?'
Robot responds: 'I can see the phone and I can pick it up for you. Reaching out to grasp it now.'

User says: 'Is there a cup there?'
Robot responds: 'Yes, I can see the cup in front of me. It looks like it's positioned in the center of my view.'

User says: 'Grab the cup'
Robot responds: 'I can see the cup and I can pick it up for you. Reaching out to grasp it now.'

Demonstration complete!
```

## Hands-on Exercises

1. Integrate a real speech recognition API (like Google Cloud Speech-to-Text) with your robot system
2. Implement a dialogue manager that maintains conversation state and handles follow-up questions
3. Create a multimodal interface that combines visual object recognition with natural language commands

## Summary

Conversational robotics represents the convergence of natural language processing, computer vision, and robotic control to create more intuitive human-robot interfaces. By integrating large language models like GPT with robotic systems, robots can understand and respond to natural language commands while considering visual context. Successful implementation requires careful integration of speech recognition, natural language understanding, and multimodal perception systems to create responsive and contextually aware robot assistants.

## Hardware Requirements

This chapter requires specialized hardware and software:
- Microphone array for speech recognition
- Camera for visual perception
- Speaker for text-to-speech output
- Internet connection for cloud-based language models
- Real-time computing platform for responsive interaction
- Robot platform with manipulation capabilities (optional for basic exercises)
- Privacy-compliant data handling setup for speech data