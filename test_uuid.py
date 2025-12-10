#!/usr/bin/env python3
"""Test script to check UUID generation in Conversation model"""

from backend.models.conversation import Conversation

# Test creating a conversation without specifying an ID
conversation = Conversation(session_id="test-defaults-456")

print(f"Conversation ID: {conversation.id}")
print(f"Conversation ID is None: {conversation.id is None}")
print(f"Session ID: {conversation.session_id}")