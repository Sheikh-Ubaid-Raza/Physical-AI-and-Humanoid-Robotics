"""
Unit tests for database models in the RAG Chatbot API
"""

import pytest
from unittest.mock import Mock, patch
from datetime import datetime
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from backend.models.conversation import Conversation
from backend.models.message import Message
from backend.utils.database import get_db_sync


@pytest.fixture
def mock_db_session():
    """
    Fixture to create a mock database session for testing
    """
    engine = create_engine("sqlite:///:memory:")
    SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
    session = SessionLocal()

    # Create tables
    Conversation.metadata.create_all(bind=engine)
    Message.metadata.create_all(bind=engine)

    yield session

    session.close()


def test_conversation_model_creation(mock_db_session):
    """
    Test creating a Conversation model instance
    """
    from datetime import datetime
    import uuid

    # Create a conversation
    conversation = Conversation(
        id=uuid.uuid4(),
        session_id="test-session-123",
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow()
    )

    # Add to session and commit
    mock_db_session.add(conversation)
    mock_db_session.commit()

    # Retrieve from database
    retrieved = mock_db_session.query(Conversation).filter_by(session_id="test-session-123").first()

    assert retrieved is not None
    assert retrieved.session_id == "test-session-123"
    assert retrieved.id == conversation.id


def test_conversation_model_defaults():
    """
    Test Conversation model default values
    """
    import uuid

    # Create conversation without specifying some fields that have defaults
    conversation = Conversation(
        session_id="test-defaults-456"
    )

    assert conversation.id is not None  # Should be auto-generated
    assert conversation.session_id == "test-defaults-456"
    assert conversation.created_at is not None
    assert conversation.updated_at is not None


def test_message_model_creation(mock_db_session):
    """
    Test creating a Message model instance
    """
    import uuid
    from datetime import datetime

    # First create a conversation to associate with the message
    conversation = Conversation(
        id=uuid.uuid4(),
        session_id="test-conversation-messages",
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow()
    )
    mock_db_session.add(conversation)
    mock_db_session.commit()

    # Create a message
    message = Message(
        id=uuid.uuid4(),
        conversation_id=conversation.id,
        role="user",
        content="Test message content",
        sources=[{"module": "Week 1", "url": "/docs/week-01/test"}],
        timestamp=datetime.utcnow()
    )

    mock_db_session.add(message)
    mock_db_session.commit()

    # Retrieve from database
    retrieved = mock_db_session.query(Message).filter_by(content="Test message content").first()

    assert retrieved is not None
    assert retrieved.role == "user"
    assert retrieved.content == "Test message content"
    assert retrieved.conversation_id == conversation.id
    assert retrieved.sources is not None


def test_message_model_required_fields():
    """
    Test that Message model validates required fields
    """
    import uuid
    from datetime import datetime

    # Create message with minimal required fields
    conversation_id = uuid.uuid4()
    message = Message(
        id=uuid.uuid4(),
        conversation_id=conversation_id,
        role="assistant",
        content="Minimal required content",
        timestamp=datetime.utcnow()
    )

    assert message.conversation_id == conversation_id
    assert message.role == "assistant"
    assert message.content == "Minimal required content"


def test_message_model_sources_optional():
    """
    Test that sources field in Message model is optional
    """
    import uuid
    from datetime import datetime

    message = Message(
        id=uuid.uuid4(),
        conversation_id=uuid.uuid4(),
        role="user",
        content="Message without sources",
        timestamp=datetime.utcnow()
        # sources field is omitted (should be None/empty)
    )

    # The message should be created successfully even without sources
    assert message.content == "Message without sources"
    # sources might be None or empty list depending on how the model handles it


def test_conversation_message_relationship(mock_db_session):
    """
    Test the relationship between Conversation and Message models
    """
    import uuid
    from datetime import datetime

    # Create a conversation
    conversation = Conversation(
        id=uuid.uuid4(),
        session_id="test-relationship",
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow()
    )
    mock_db_session.add(conversation)
    mock_db_session.commit()

    # Create multiple messages for the conversation
    message1 = Message(
        id=uuid.uuid4(),
        conversation_id=conversation.id,
        role="user",
        content="First message",
        timestamp=datetime.utcnow()
    )

    message2 = Message(
        id=uuid.uuid4(),
        conversation_id=conversation.id,
        role="assistant",
        content="Response to first message",
        timestamp=datetime.utcnow()
    )

    mock_db_session.add(message1)
    mock_db_session.add(message2)
    mock_db_session.commit()

    # Test the relationship
    retrieved_conversation = mock_db_session.query(Conversation).filter_by(session_id="test-relationship").first()
    assert len(retrieved_conversation.messages) == 2

    # Verify the messages belong to the correct conversation
    for msg in retrieved_conversation.messages:
        assert msg.conversation_id == conversation.id


def test_conversation_timestamps_auto_update(mock_db_session):
    """
    Test that Conversation timestamps are automatically updated
    """
    import uuid
    from datetime import datetime

    # Create conversation
    conversation = Conversation(
        id=uuid.uuid4(),
        session_id="test-timestamps",
        created_at=datetime.utcnow()
    )
    mock_db_session.add(conversation)
    mock_db_session.commit()

    original_updated_at = conversation.updated_at

    # Update the conversation (even just touching it should update updated_at)
    conversation.session_id = "updated-test-timestamps"
    mock_db_session.commit()

    # Refresh from database to get updated values
    mock_db_session.refresh(conversation)

    # Verify that updated_at has changed
    assert conversation.updated_at > original_updated_at
    assert conversation.session_id == "updated-test-timestamps"


def test_message_timestamps():
    """
    Test Message model timestamp behavior
    """
    import uuid
    from datetime import datetime

    message = Message(
        id=uuid.uuid4(),
        conversation_id=uuid.uuid4(),
        role="user",
        content="Test message with timestamp",
        timestamp=datetime.utcnow()
    )

    assert message.timestamp is not None
    assert isinstance(message.timestamp, datetime)


def test_conversation_repr():
    """
    Test the string representation of Conversation model
    """
    import uuid

    conversation = Conversation(
        id=uuid.uuid4(),
        session_id="test-repr-123"
    )

    repr_str = repr(conversation)
    assert "Conversation" in repr_str
    assert "test-repr-123" in repr_str


def test_message_repr():
    """
    Test the string representation of Message model
    """
    import uuid

    message = Message(
        id=uuid.uuid4(),
        conversation_id=uuid.uuid4(),
        role="user",
        content="Test content for repr"
    )

    repr_str = repr(message)
    assert "Message" in repr_str
    assert "user" in repr_str
    assert "Test content for repr" in repr_str


if __name__ == "__main__":
    pytest.main([__file__])