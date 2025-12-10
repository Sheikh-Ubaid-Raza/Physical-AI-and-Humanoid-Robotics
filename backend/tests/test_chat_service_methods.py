"""
Unit tests for ChatService methods in the RAG Chatbot API
These tests focus on improving code coverage by testing methods in isolation
"""
import pytest
from unittest.mock import Mock, patch, MagicMock
from datetime import datetime, timedelta
from backend.services.chat_service import ChatService


@pytest.fixture
def mock_db():
    """
    Mock database session for testing
    """
    return Mock()


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
def chat_service(mock_db, mock_rag_service):
    """
    Create a ChatService instance for testing with mocked dependencies
    """
    with patch('backend.services.chat_service.get_rag_service', return_value=mock_rag_service):
        service = ChatService(mock_db)
        return service


def test_chat_service_initialization(mock_db, mock_rag_service):
    """
    Test ChatService initialization
    """
    with patch('backend.services.chat_service.get_rag_service', return_value=mock_rag_service):
        service = ChatService(mock_db)

        assert service.db == mock_db
        assert service.rag_service == mock_rag_service


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


def test_update_message_storage_with_context_tracking(chat_service):
    """
    Test update_message_storage_with_context_tracking method
    """
    # This method currently just passes, but we should test it
    result = chat_service.update_message_storage_with_context_tracking("test-conversation-id")
    # Method should not raise an exception and return None (implicit)
    assert result is None


def test_validate_conversation_context_error_handling(chat_service):
    """
    Test error handling in validate_conversation_context
    """
    # Mock the database query to raise an exception
    mock_query = Mock()
    mock_query.filter.side_effect = Exception("DB Error")
    chat_service.db.query.return_value = mock_query

    with pytest.raises(Exception):
        chat_service.validate_conversation_context("test-id")


def test_cleanup_conversation_context_error_handling(chat_service):
    """
    Test error handling in cleanup_conversation_context
    """
    # Mock the database query to raise an exception
    chat_service.db.query.side_effect = Exception("DB Error")

    with pytest.raises(Exception):
        chat_service.cleanup_conversation_context("test-id")


def test_get_conversation_history_error_handling(chat_service):
    """
    Test error handling in get_conversation_history
    """
    # Mock the database query to raise an exception
    chat_service.db.query.side_effect = Exception("DB Error")

    with pytest.raises(Exception):
        chat_service.get_conversation_history("test-session")


def test_start_new_conversation_error_handling(chat_service):
    """
    Test error handling in start_new_conversation
    """
    from backend.utils.error_handlers import APIError
    with patch('backend.services.session_service.get_or_create_session', side_effect=Exception("DB Error")):
        with pytest.raises(APIError):
            chat_service.start_new_conversation("new-session")


def test_end_conversation_error_handling(chat_service):
    """
    Test error handling in end_conversation
    """
    # Mock the database query to raise an exception
    mock_query = Mock()
    mock_query.filter.side_effect = Exception("DB Error")
    chat_service.db.query.return_value = mock_query

    with pytest.raises(Exception):
        chat_service.end_conversation("test-id")


def test_get_conversation_context_window_empty_result(chat_service):
    """
    Test get_conversation_context_window with empty query result
    """
    # Mock the database query chain properly
    mock_query_result = Mock()
    mock_filtered_query = Mock()
    mock_ordered_query = Mock()
    mock_limited_query = Mock()

    # Set up the chain: query -> filter -> order_by -> limit -> all
    mock_query_result.filter.return_value = mock_filtered_query
    mock_filtered_query.order_by.return_value = mock_ordered_query
    mock_ordered_query.limit.return_value = mock_limited_query
    mock_limited_query.all.return_value = []

    # Mock the query call to return our chain when Message is passed
    chat_service.db.query.return_value = mock_query_result

    context = chat_service.get_conversation_context_window("test-conversation-id", window_size=5)

    assert context == []


def test_get_conversation_context_window_with_messages(chat_service):
    """
    Test get_conversation_context_window with messages
    """
    # Create mock messages
    mock_msg1 = Mock()
    mock_msg1.role = "user"
    mock_msg1.content = "Hello"
    mock_msg1.timestamp = datetime.utcnow()

    mock_msg2 = Mock()
    mock_msg2.role = "assistant"
    mock_msg2.content = "Hi there"
    mock_msg2.timestamp = datetime.utcnow()

    # Mock the database query chain properly
    mock_query_result = Mock()
    mock_filtered_query = Mock()
    mock_ordered_query = Mock()
    mock_limited_query = Mock()

    # Set up the chain: query -> filter -> order_by -> limit -> all
    mock_query_result.filter.return_value = mock_filtered_query
    mock_filtered_query.order_by.return_value = mock_ordered_query
    mock_ordered_query.limit.return_value = mock_limited_query
    mock_limited_query.all.return_value = [mock_msg1, mock_msg2]

    # Mock the query call to return our chain when Message is passed
    chat_service.db.query.return_value = mock_query_result

    context = chat_service.get_conversation_context_window("test-conversation-id", window_size=5)

    assert len(context) == 2
    assert context[0]["role"] == "user"
    assert context[0]["content"] == "Hello"
    assert context[1]["role"] == "assistant"
    assert context[1]["content"] == "Hi there"


def test_get_conversation_context_window_with_none_timestamp(chat_service):
    """
    Test get_conversation_context_window with message timestamp as None
    """
    # Create mock messages with None timestamp
    mock_msg = Mock()
    mock_msg.role = "user"
    mock_msg.content = "Hello"
    mock_msg.timestamp = None

    # Mock the database query chain properly
    mock_query_result = Mock()
    mock_filtered_query = Mock()
    mock_ordered_query = Mock()
    mock_limited_query = Mock()

    # Set up the chain: query -> filter -> order_by -> limit -> all
    mock_query_result.filter.return_value = mock_filtered_query
    mock_filtered_query.order_by.return_value = mock_ordered_query
    mock_ordered_query.limit.return_value = mock_limited_query
    mock_limited_query.all.return_value = [mock_msg]

    # Mock the query call to return our chain when Message is passed
    chat_service.db.query.return_value = mock_query_result

    context = chat_service.get_conversation_context_window("test-conversation-id", window_size=5)

    assert len(context) == 1
    assert context[0]["role"] == "user"
    assert context[0]["content"] == "Hello"
    assert context[0]["timestamp"] is None


def test_validate_conversation_context_invalid_conversation(chat_service):
    """
    Test validate_conversation_context with invalid conversation
    """
    # Mock the database query to return None (no conversation found)
    mock_query = Mock()
    mock_query.filter.return_value = mock_query
    mock_query.first.return_value = None
    chat_service.db.query.return_value = mock_query

    result = chat_service.validate_conversation_context("non-existent-id")

    assert result is False


def test_validate_conversation_context_old_conversation(chat_service):
    """
    Test validate_conversation_context with old conversation
    """
    # Create a mock conversation with old timestamp
    old_time = datetime.utcnow() - timedelta(hours=25)  # More than 24 hours old
    mock_conversation = Mock()
    mock_conversation.updated_at = old_time

    # Mock the database query
    mock_query = Mock()
    mock_query.filter.return_value = mock_query
    mock_query.first.return_value = mock_conversation
    chat_service.db.query.return_value = mock_query

    result = chat_service.validate_conversation_context("old-conversation-id")

    # Should return False because conversation is older than 24 hours
    assert result is False


def test_validate_conversation_context_valid_conversation(chat_service):
    """
    Test validate_conversation_context with valid conversation
    """
    # Create a mock conversation with recent timestamp
    recent_time = datetime.utcnow() - timedelta(hours=1)  # 1 hour old
    mock_conversation = Mock()
    mock_conversation.updated_at = recent_time

    # Mock the database query
    mock_query = Mock()
    mock_query.filter.return_value = mock_query
    mock_query.first.return_value = mock_conversation
    chat_service.db.query.return_value = mock_query

    result = chat_service.validate_conversation_context("valid-conversation-id")

    # Should return True because conversation is recent
    assert result is True


def test_get_conversation_history_empty(chat_service):
    """
    Test get_conversation_history with no conversation found
    """
    # Mock the database query to return None for conversation
    mock_conversation_query = Mock()
    mock_conversation_query.filter.return_value = mock_conversation_query
    mock_conversation_query.first.return_value = None

    # Mock the message query separately
    mock_message_query = Mock()

    # Make db.query return different mock objects based on what's being queried
    def query_side_effect(model_class):
        if model_class.__name__ == 'Conversation':
            return mock_conversation_query
        else:  # For Message
            return mock_message_query

    chat_service.db.query.side_effect = query_side_effect

    result = chat_service.get_conversation_history("non-existent-session")

    assert result == []


def test_get_conversation_history_with_messages(chat_service):
    """
    Test get_conversation_history with messages
    """
    # Create mock conversation
    mock_conversation = Mock()
    mock_conversation.id = "test-conversation-id"

    # Create mock messages
    mock_msg1 = Mock()
    mock_msg1.id = "msg1"
    mock_msg1.role = "user"
    mock_msg1.content = "Hello"
    mock_msg1.timestamp = datetime.utcnow()
    mock_msg1.sources = []

    mock_msg2 = Mock()
    mock_msg2.id = "msg2"
    mock_msg2.role = "assistant"
    mock_msg2.content = "Hi there"
    mock_msg2.timestamp = datetime.utcnow()
    mock_msg2.sources = [{"module": "Test", "url": "/test"}]

    # Mock the database queries
    mock_conversation_query = Mock()
    mock_conversation_query.filter.return_value = mock_conversation_query
    mock_conversation_query.first.return_value = mock_conversation

    mock_message_query = Mock()
    mock_message_query.filter.return_value = mock_message_query
    mock_message_query.order_by.return_value = mock_message_query
    mock_message_query.all.return_value = [mock_msg1, mock_msg2]

    def query_side_effect(model_class):
        if model_class.__name__ == 'Conversation':
            return mock_conversation_query
        else:  # For Message
            return mock_message_query

    chat_service.db.query.side_effect = query_side_effect

    result = chat_service.get_conversation_history("test-session")

    assert len(result) == 2
    assert result[0]["id"] == "msg1"
    assert result[0]["role"] == "user"
    assert result[0]["content"] == "Hello"
    assert result[1]["id"] == "msg2"
    assert result[1]["role"] == "assistant"
    assert result[1]["content"] == "Hi there"


def test_start_new_conversation_success(chat_service):
    """
    Test starting a new conversation successfully
    """
    mock_conversation = Mock()
    mock_conversation.id = "new-conversation-id"

    with patch('backend.services.session_service.get_or_create_session', return_value=mock_conversation):
        conversation_id = chat_service.start_new_conversation("new-session")

        assert conversation_id == "new-conversation-id"


def test_end_conversation_success(chat_service):
    """
    Test ending a conversation successfully
    """
    # Create mock conversation
    mock_conversation = Mock()

    # Mock the database query
    mock_query = Mock()
    mock_query.filter.return_value = mock_query
    mock_query.first.return_value = mock_conversation
    chat_service.db.query.return_value = mock_query

    # Call the method
    chat_service.end_conversation("test-conversation-id")

    # Verify that updated_at was set and commit was called
    assert mock_conversation.updated_at is not None
    chat_service.db.commit.assert_called_once()


def test_end_conversation_nonexistent(chat_service):
    """
    Test ending a non-existent conversation (should not raise error)
    """
    # Mock the database query to return None
    mock_query = Mock()
    mock_query.filter.return_value = mock_query
    mock_query.first.return_value = None
    chat_service.db.query.return_value = mock_query

    # This should not raise an error even if conversation doesn't exist
    chat_service.end_conversation("non-existent-id")

    # Commit should not be called since no conversation was found
    chat_service.db.commit.assert_not_called()


def test_process_message_error_handling(chat_service):
    """
    Test error handling in process_message
    """
    with patch('backend.services.session_service.get_or_create_session', side_effect=Exception("DB Error")):
        with pytest.raises(Exception):
            chat_service.process_message(
                session_id="test-session",
                user_message="Test message"
            )


def test_process_message_success_path(chat_service):
    """
    Test the process_message method with mocked dependencies
    """
    # Mock conversation
    mock_conversation = Mock()
    mock_conversation.id = "test-conversation-id"

    # Mock message objects
    mock_user_msg = Mock()
    mock_user_msg.id = "user-msg-id"
    mock_user_msg.role = "user"

    mock_assistant_msg = Mock()
    mock_assistant_msg.id = "assistant-msg-id"
    mock_assistant_msg.role = "assistant"

    # Mock session service
    with patch('backend.services.session_service.get_or_create_session', return_value=mock_conversation):
        # Mock the database operations
        with patch.object(chat_service.db, 'add'), \
             patch.object(chat_service.db, 'commit'), \
             patch.object(chat_service.db, 'refresh', side_effect=lambda x: setattr(x, 'id', str(mock_user_msg.id)) if hasattr(x, 'role') and x.role == 'user' else setattr(x, 'id', str(mock_assistant_msg.id)) if hasattr(x, 'role') else None):

            result = chat_service.process_message(
                session_id="test-session",
                user_message="Hello, how are you?"
            )

            # Verify the result structure
            assert "response" in result
            assert "sources" in result
            assert "conversation_id" in result
            assert "message_id" in result
            assert result["conversation_id"] == "test-conversation-id"


def test_get_chat_service_function():
    """
    Test the get_chat_service function
    """
    mock_db = Mock()
    mock_rag_service = Mock()

    with patch('backend.services.chat_service.get_rag_service', return_value=mock_rag_service):
        from backend.services.chat_service import get_chat_service
        service = get_chat_service(mock_db)

        assert isinstance(service, ChatService)
        assert service.db == mock_db
        assert service.rag_service == mock_rag_service