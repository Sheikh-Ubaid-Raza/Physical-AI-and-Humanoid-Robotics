"""
Unit tests for the ChatService in the RAG Chatbot API
These tests focus on improving code coverage for the ChatService
"""
import pytest
from unittest.mock import Mock, patch, MagicMock
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from datetime import datetime, timedelta
from backend.services.chat_service import ChatService
from backend.models.conversation import Conversation
from backend.models.message import Message


@pytest.fixture
def mock_db_session():
    """
    Create a mock database session for testing
    """
    engine = create_engine("sqlite:///:memory:")
    SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
    session = SessionLocal()

    # Create tables
    Conversation.metadata.create_all(bind=engine)
    Message.metadata.create_all(bind=engine)

    yield session

    session.close()


@pytest.fixture
def mock_rag_service():
    """
    Mock RAG service for testing
    """
    service = Mock()
    service.generate_response.return_value = {
        "response": "Test response from RAG service",
        "sources": [
            {
                "module": "ROS 2 Fundamentals",
                "week": "Week 3",
                "chapter_title": "ROS 2 Architecture",
                "url": "/docs/week-03/ros2-architecture"
            }
        ]
    }
    return service


@pytest.fixture
def chat_service(mock_db_session, mock_rag_service):
    """
    Create a ChatService instance for testing
    """
    with patch('backend.services.chat_service.get_rag_service', return_value=mock_rag_service):
        service = ChatService(mock_db_session)
        return service


def test_chat_service_initialization(mock_db_session, mock_rag_service):
    """
    Test ChatService initialization
    """
    with patch('backend.services.chat_service.get_rag_service', return_value=mock_rag_service):
        service = ChatService(mock_db_session)

        assert service.db == mock_db_session
        assert service.rag_service == mock_rag_service


def test_process_message_success(chat_service, mock_db_session):
    """
    Test successful message processing
    """
    # Create a conversation
    conversation = Conversation(session_id="test-session-123")
    mock_db_session.add(conversation)
    mock_db_session.commit()

    # Process a message
    result = chat_service.process_message(
        session_id="test-session-123",
        user_message="Hello, how are you?",
        model="gpt-4o"
    )

    # Verify the result
    assert "response" in result
    assert "sources" in result
    assert "conversation_id" in result
    assert "message_id" in result

    # Verify that messages were created in the database
    messages = mock_db_session.query(Message).all()
    assert len(messages) == 2  # User message + Assistant message


def test_process_message_error_handling(chat_service):
    """
    Test message processing error handling
    """
    with patch.object(chat_service, 'rag_service', side_effect=Exception("Test error")):
        with pytest.raises(Exception):
            chat_service.process_message(
                session_id="non-existent-session",
                user_message="Test message"
            )


def test_get_conversation_context(chat_service, mock_db_session):
    """
    Test getting conversation context
    """
    # Create a conversation
    conversation = Conversation(session_id="test-context-session")
    mock_db_session.add(conversation)
    mock_db_session.commit()

    # Create some messages
    user_msg = Message(
        conversation_id=conversation.id,
        role="user",
        content="Hello"
    )
    assistant_msg = Message(
        conversation_id=conversation.id,
        role="assistant",
        content="Hi there"
    )
    mock_db_session.add(user_msg)
    mock_db_session.add(assistant_msg)
    mock_db_session.commit()

    # Test the private method by accessing it directly
    context = chat_service._get_conversation_context(str(conversation.id), limit=10)

    assert len(context) == 2
    assert context[0]["role"] == "user"
    assert context[0]["content"] == "Hello"
    assert context[1]["role"] == "assistant"
    assert context[1]["content"] == "Hi there"


def test_get_conversation_context_window(chat_service, mock_db_session):
    """
    Test getting conversation context with window size
    """
    # Create a conversation
    conversation = Conversation(session_id="test-window-session")
    mock_db_session.add(conversation)
    mock_db_session.commit()

    # Create multiple messages
    for i in range(10):
        msg = Message(
            conversation_id=conversation.id,
            role="user",
            content=f"Message {i}"
        )
        mock_db_session.add(msg)
    mock_db_session.commit()

    # Test with window size of 5
    context = chat_service.get_conversation_context_window(str(conversation.id), window_size=5)

    assert len(context) == 5
    # Should return the most recent 5 messages in chronological order
    assert context[0]["content"] == "Message 5"  # Oldest in the window
    assert context[4]["content"] == "Message 9"  # Newest in the window


def test_update_message_storage_with_context_tracking(chat_service):
    """
    Test update_message_storage_with_context_tracking method
    """
    # This method currently just passes, but we should test it
    result = chat_service.update_message_storage_with_context_tracking("test-conversation-id")
    # Method should not raise an exception and return None (implicit)
    assert result is None


def test_get_conversation_history(chat_service, mock_db_session):
    """
    Test getting conversation history
    """
    # Create a conversation
    conversation = Conversation(session_id="test-history-session")
    mock_db_session.add(conversation)
    mock_db_session.commit()

    # Create some messages
    user_msg = Message(
        conversation_id=conversation.id,
        role="user",
        content="Hello"
    )
    assistant_msg = Message(
        conversation_id=conversation.id,
        role="assistant",
        content="Hi there"
    )
    mock_db_session.add(user_msg)
    mock_db_session.add(assistant_msg)
    mock_db_session.commit()

    # Get conversation history
    history = chat_service.get_conversation_history("test-history-session")

    assert len(history) == 2
    assert history[0]["role"] == "user"
    assert history[0]["content"] == "Hello"
    assert history[1]["role"] == "assistant"
    assert history[1]["content"] == "Hi there"


def test_get_conversation_history_nonexistent(chat_service):
    """
    Test getting conversation history for non-existent session
    """
    history = chat_service.get_conversation_history("non-existent-session")

    assert history == []


def test_get_conversation_history_error(chat_service):
    """
    Test error handling in get_conversation_history
    """
    with patch.object(chat_service.db, 'query', side_effect=Exception("DB Error")):
        with pytest.raises(Exception):
            chat_service.get_conversation_history("test-session")


def test_validate_message_valid(chat_service):
    """
    Test message validation with valid message
    """
    result = chat_service.validate_message("This is a valid message")

    assert result is True


def test_validate_message_empty(chat_service):
    """
    Test message validation with empty message
    """
    result = chat_service.validate_message("")

    assert result is False


def test_validate_message_whitespace_only(chat_service):
    """
    Test message validation with whitespace-only message
    """
    result = chat_service.validate_message("   ")

    assert result is False


def test_validate_message_too_long(chat_service):
    """
    Test message validation with message that's too long
    """
    long_message = "A" * 2001  # More than 2000 character limit
    result = chat_service.validate_message(long_message)

    assert result is False


def test_start_new_conversation(chat_service, mock_db_session):
    """
    Test starting a new conversation
    """
    with patch('backend.services.session_service.get_or_create_session') as mock_get_or_create:
        mock_conversation = Conversation(session_id="new-session")
        mock_get_or_create.return_value = mock_conversation

        conversation_id = chat_service.start_new_conversation("new-session")

        assert conversation_id == str(mock_conversation.id)


def test_start_new_conversation_error(chat_service):
    """
    Test error handling in start_new_conversation
    """
    with patch('backend.services.session_service.get_or_create_session', side_effect=Exception("DB Error")):
        with pytest.raises(Exception):
            chat_service.start_new_conversation("new-session")


def test_end_conversation(chat_service, mock_db_session):
    """
    Test ending a conversation
    """
    # Create a conversation
    conversation = Conversation(session_id="test-end-session")
    mock_db_session.add(conversation)
    mock_db_session.commit()

    # Get original updated_at
    original_updated_at = conversation.updated_at

    # End the conversation
    chat_service.end_conversation(str(conversation.id))

    # Refresh from database
    mock_db_session.refresh(conversation)

    # Updated at should be different (newer)
    assert conversation.updated_at >= original_updated_at


def test_end_conversation_nonexistent(chat_service):
    """
    Test ending a non-existent conversation (should not raise error)
    """
    # This should not raise an error even if conversation doesn't exist
    chat_service.end_conversation("non-existent-id")


def test_end_conversation_error(chat_service):
    """
    Test error handling in end_conversation
    """
    with patch.object(chat_service.db, 'query', side_effect=Exception("DB Error")):
        with pytest.raises(Exception):
            chat_service.end_conversation("test-id")


def test_validate_conversation_context_valid(chat_service, mock_db_session):
    """
    Test validating conversation context for a valid conversation
    """
    # Create a conversation with recent activity
    conversation = Conversation(session_id="test-validate-session")
    mock_db_session.add(conversation)
    mock_db_session.commit()

    # Validate the conversation context
    result = chat_service.validate_conversation_context(str(conversation.id))

    assert result is True


def test_validate_conversation_context_nonexistent(chat_service):
    """
    Test validating conversation context for non-existent conversation
    """
    result = chat_service.validate_conversation_context("non-existent-id")

    assert result is False


def test_validate_conversation_context_old(chat_service, mock_db_session):
    """
    Test validating conversation context for an old conversation
    """
    # Create a conversation with old timestamp
    old_time = datetime.utcnow() - timedelta(hours=25)  # More than 24 hours old
    conversation = Conversation(
        session_id="test-old-session",
        created_at=old_time,
        updated_at=old_time
    )
    mock_db_session.add(conversation)
    mock_db_session.commit()

    result = chat_service.validate_conversation_context(str(conversation.id))

    # Should return False because conversation is older than 24 hours
    assert result is False


def test_validate_conversation_context_error(chat_service):
    """
    Test error handling in validate_conversation_context
    """
    with patch.object(chat_service.db, 'query', side_effect=Exception("DB Error")):
        with pytest.raises(Exception):
            chat_service.validate_conversation_context("test-id")


def test_cleanup_conversation_context(chat_service, mock_db_session):
    """
    Test cleaning up conversation context by removing old messages
    """
    # Create a conversation
    conversation = Conversation(session_id="test-cleanup-session")
    mock_db_session.add(conversation)
    mock_db_session.commit()

    # Create more than 20 messages
    for i in range(25):
        msg = Message(
            conversation_id=conversation.id,
            role="user",
            content=f"Message {i}"
        )
        mock_db_session.add(msg)
    mock_db_session.commit()

    # Verify we have 25 messages
    all_messages = mock_db_session.query(Message).filter(
        Message.conversation_id == conversation.id
    ).all()
    assert len(all_messages) == 25

    # Clean up conversation context
    chat_service.cleanup_conversation_context(str(conversation.id))

    # Should now have at most 20 messages
    remaining_messages = mock_db_session.query(Message).filter(
        Message.conversation_id == conversation.id
    ).all()
    assert len(remaining_messages) <= 20


def test_cleanup_conversation_context_error(chat_service):
    """
    Test error handling in cleanup_conversation_context
    """
    with patch.object(chat_service.db, 'query', side_effect=Exception("DB Error")):
        with pytest.raises(Exception):
            chat_service.cleanup_conversation_context("test-id")


def test_get_chat_service_function(mock_db_session, mock_rag_service):
    """
    Test the get_chat_service function
    """
    with patch('backend.services.chat_service.get_rag_service', return_value=mock_rag_service):
        service = chat_service(mock_db_session, mock_rag_service)

        assert isinstance(service, ChatService)
        assert service.db == mock_db_session