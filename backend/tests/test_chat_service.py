"""
Unit tests for the Chat Service in the RAG Chatbot API
"""

import pytest
from unittest.mock import Mock, patch, MagicMock
from datetime import datetime, timezone
from backend.services.chat_service import ChatService
from backend.models.conversation import Conversation
from backend.models.message import Message
from backend.services.rag_service import RAGService


@pytest.fixture
def mock_db_session():
    """
    Mock database session for testing
    """
    session = Mock()
    session.query.return_value.filter.return_value.first.return_value = None
    session.query.return_value.filter.return_value.all.return_value = []
    return session


@pytest.fixture
def mock_rag_service():
    """
    Mock RAG service for testing
    """
    rag_service = Mock(spec=RAGService)
    rag_service.generate_response.return_value = {
        "response": "Test response from RAG service",
        "sources": [
            {
                "module": "Test Module",
                "week": "Week 1",
                "chapter_title": "Test Chapter",
                "url": "/docs/test-chapter"
            }
        ]
    }
    return rag_service


@pytest.fixture
def chat_service(mock_db_session, mock_rag_service):
    """
    Create a ChatService instance with mocked dependencies
    """
    service = ChatService(mock_db_session)
    service.rag_service = mock_rag_service
    return service


def test_chat_service_initialization(mock_db_session):
    """
    Test that ChatService initializes properly with a database session
    """
    service = ChatService(mock_db_session)

    assert service.db == mock_db_session
    assert service.rag_service is not None


def test_process_message_creates_conversation_if_not_exists(chat_service, mock_db_session):
    """
    Test that process_message creates a new conversation if one doesn't exist
    """
    # Mock that no conversation exists initially
    mock_db_session.query.return_value.filter.return_value.first.return_value = None

    # Mock the conversation creation
    mock_conversation = Mock(spec=Conversation)
    mock_conversation.id = "test-conversation-id"
    mock_conversation.session_id = "test-session-123"
    mock_db_session.merge.return_value = mock_conversation

    with patch('backend.services.chat_service.get_or_create_session') as mock_get_session:
        mock_get_session.return_value = mock_conversation

        result = chat_service.process_message("test-session-123", "Hello, world!")

        # Verify that a conversation was created/retrieved
        mock_get_session.assert_called_once_with("test-session-123", mock_db_session)


def test_process_message_creates_user_message(chat_service, mock_db_session):
    """
    Test that process_message creates a user message record
    """
    # Mock conversation
    mock_conversation = Mock(spec=Conversation)
    mock_conversation.id = "test-conversation-id"
    mock_conversation.session_id = "test-session-123"

    # Mock the session retrieval
    with patch('backend.services.chat_service.get_or_create_session') as mock_get_session:
        mock_get_session.return_value = mock_conversation

        # Call the method
        result = chat_service.process_message("test-session-123", "Hello, world!")

        # Verify that a user message was added to the session
        assert mock_db_session.add.call_count >= 1
        # Check that a Message object was added
        added_obj = mock_db_session.add.call_args[0][0]
        assert isinstance(added_obj, Message)
        assert added_obj.role == "user"
        assert added_obj.content == "Hello, world!"


def test_process_message_calls_rag_service(chat_service, mock_db_session):
    """
    Test that process_message calls the RAG service to generate a response
    """
    # Mock conversation
    mock_conversation = Mock(spec=Conversation)
    mock_conversation.id = "test-conversation-id"
    mock_conversation.session_id = "test-session-123"

    with patch('backend.services.chat_service.get_or_create_session') as mock_get_session:
        mock_get_session.return_value = mock_conversation

        # Call the method
        result = chat_service.process_message("test-session-123", "Hello, world!")

        # Verify that RAG service was called
        chat_service.rag_service.generate_response.assert_called_once()


def test_process_message_returns_proper_response_format(chat_service, mock_db_session):
    """
    Test that process_message returns response in the expected format
    """
    # Mock conversation
    mock_conversation = Mock(spec=Conversation)
    mock_conversation.id = "test-conversation-id"
    mock_conversation.session_id = "test-session-123"

    with patch('backend.services.chat_service.get_or_create_session') as mock_get_session:
        mock_get_session.return_value = mock_conversation

        # Call the method
        result = chat_service.process_message("test-session-123", "Hello, world!")

        # Verify response structure
        assert "response" in result
        assert "sources" in result
        assert "conversation_id" in result
        assert "message_id" in result

        # Verify content
        assert result["response"] == "Test response from RAG service"
        assert len(result["sources"]) == 1
        assert result["conversation_id"] == "test-conversation-id"


def test_process_message_handles_empty_message(chat_service):
    """
    Test that process_message properly handles empty messages
    """
    with pytest.raises(ValueError):
        chat_service.process_message("test-session", "")


def test_process_message_handles_long_message(chat_service):
    """
    Test that process_message properly handles long messages
    """
    long_message = "A" * 2001  # Longer than the 2000 char limit

    with pytest.raises(ValueError):
        chat_service.process_message("test-session", long_message)


def test_get_conversation_history_success(chat_service, mock_db_session):
    """
    Test that get_conversation_history returns the conversation history
    """
    # Mock conversation
    mock_conversation = Mock(spec=Conversation)
    mock_conversation.id = "test-conversation-id"

    # Mock messages
    mock_message1 = Mock(spec=Message)
    mock_message1.id = "msg1"
    mock_message1.role = "user"
    mock_message1.content = "Hello"
    mock_message1.timestamp = datetime.now(timezone.utc)
    mock_message1.sources = []

    mock_message2 = Mock(spec=Message)
    mock_message2.id = "msg2"
    mock_message2.role = "assistant"
    mock_message2.content = "Hi there"
    mock_message2.timestamp = datetime.now(timezone.utc)
    mock_message2.sources = [{"module": "Test", "url": "/test"}]

    # Configure mocks
    mock_db_session.query.return_value.filter.return_value.first.return_value = mock_conversation
    mock_db_session.query.return_value.filter.return_value.order_by.return_value.all.return_value = [
        mock_message1, mock_message2
    ]

    # Call the method
    result = chat_service.get_conversation_history("test-session-123")

    # Verify the result
    assert len(result) == 2
    assert result[0]["role"] == "user"
    assert result[0]["content"] == "Hello"
    assert result[1]["role"] == "assistant"
    assert result[1]["content"] == "Hi there"


def test_get_conversation_history_no_conversation(chat_service, mock_db_session):
    """
    Test that get_conversation_history returns empty list when no conversation exists
    """
    # Configure mock to return None (no conversation found)
    mock_db_session.query.return_value.filter.return_value.first.return_value = None

    # Call the method
    result = chat_service.get_conversation_history("nonexistent-session")

    # Verify the result
    assert result == []


def test_validate_message_empty(chat_service):
    """
    Test that validate_message returns False for empty messages
    """
    result = chat_service.validate_message("")
    assert result is False

    result = chat_service.validate_message("   ")  # Just whitespace
    assert result is False

    result = chat_service.validate_message(None)  # None message
    assert result is False


def test_validate_message_valid(chat_service):
    """
    Test that validate_message returns True for valid messages
    """
    result = chat_service.validate_message("Hello, this is a valid message.")
    assert result is True


def test_validate_message_too_long(chat_service):
    """
    Test that validate_message returns False for messages exceeding length limit
    """
    long_message = "A" * 2001  # 1 character over the 2000 limit
    result = chat_service.validate_message(long_message)
    assert result is False


def test_validate_message_within_limit(chat_service):
    """
    Test that validate_message returns True for messages within length limit
    """
    valid_message = "A" * 2000  # Exactly at the 2000 limit
    result = chat_service.validate_message(valid_message)
    assert result is True


def test_start_new_conversation(chat_service, mock_db_session):
    """
    Test that start_new_conversation creates a new conversation
    """
    mock_conversation = Mock(spec=Conversation)
    mock_conversation.id = "new-conversation-id"

    with patch('backend.services.chat_service.get_or_create_session') as mock_get_session:
        mock_get_session.return_value = mock_conversation

        result = chat_service.start_new_conversation("test-session-123")

        assert result == "new-conversation-id"
        mock_get_session.assert_called_once_with("test-session-123", mock_db_session)


def test_end_conversation_updates_timestamp(chat_service, mock_db_session):
    """
    Test that end_conversation updates the conversation's updated_at timestamp
    """
    # Mock conversation
    mock_conversation = Mock(spec=Conversation)
    mock_conversation.id = "test-conversation-id"

    # Configure mock to return the conversation
    mock_db_session.query.return_value.filter.return_value.first.return_value = mock_conversation

    # Call the method
    chat_service.end_conversation("test-conversation-id")

    # Verify that the conversation's updated_at was set and session committed
    assert mock_conversation.updated_at is not None
    mock_db_session.commit.assert_called()


def test_end_conversation_nonexistent(chat_service, mock_db_session):
    """
    Test that end_conversation handles nonexistent conversations gracefully
    """
    # Configure mock to return None (no conversation found)
    mock_db_session.query.return_value.filter.return_value.first.return_value = None

    # Should not raise an exception
    chat_service.end_conversation("nonexistent-conversation-id")

    # Verify that commit was still called (even though no conversation was found)
    mock_db_session.commit.assert_called()


def test_process_message_error_handling(chat_service, mock_db_session):
    """
    Test that process_message handles errors properly
    """
    # Mock RAG service to raise an exception
    chat_service.rag_service.generate_response.side_effect = Exception("Test error")

    # Mock conversation
    mock_conversation = Mock(spec=Conversation)
    mock_conversation.id = "test-conversation-id"
    mock_conversation.session_id = "test-session-123"

    with patch('backend.services.chat_service.get_or_create_session') as mock_get_session:
        mock_get_session.return_value = mock_conversation

        # Should raise an APIError
        with pytest.raises(Exception) as exc_info:
            chat_service.process_message("test-session-123", "Hello, world!")

        assert "Error processing message" in str(exc_info.value)


if __name__ == "__main__":
    pytest.main([__file__])