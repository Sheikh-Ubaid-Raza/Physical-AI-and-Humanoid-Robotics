#!/usr/bin/env python3
"""Test script to check UUID generation in Conversation model"""

# Import both models to resolve circular dependency
from backend.models.conversation import Conversation
from backend.models.message import Message
import uuid

# Test creating a conversation without specifying an ID
print("Creating conversation object...")
conversation = Conversation(session_id="test-defaults-456")

print(f"Conversation ID before: {conversation.id}")
print(f"Conversation ID is None: {conversation.id is None}")

# Now let's try with a database session
print("\nTesting with database session...")

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from backend.models.conversation import Base

# Create an in-memory database for testing
engine = create_engine("sqlite:///:memory:")
Base.metadata.create_all(bind=engine)
Session = sessionmaker(bind=engine)
session = Session()

# Create conversation through the session
conv = Conversation(session_id="test-with-session")
print(f"Conversation ID before adding to session: {conv.id}")

session.add(conv)
session.flush()  # This should trigger default value generation
print(f"Conversation ID after flush: {conv.id}")

session.commit()
print(f"Conversation ID after commit: {conv.id}")