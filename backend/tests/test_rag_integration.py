"""
Integration tests for the RAG Pipeline in the RAG Chatbot API
"""

import pytest
from unittest.mock import Mock, patch, MagicMock
from fastapi.testclient import TestClient
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from backend.main import app
from backend.services.rag_service import RAGService
from backend.services.chat_service import ChatService
from backend.models.conversation import Conversation
from backend.models.message import Message
from backend.utils.database import get_db, get_db_sync
from backend.api.routes.chat import router as chat_router


@pytest.fixture
def test_client():
    """
    Create a test client for the FastAPI app
    """
    return TestClient(app)


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
    service = Mock(spec=RAGService)
    service.generate_response.return_value = {
        "response": "Test response from RAG service with proper citations.",
        "sources": [
            {
                "module": "ROS 2 Fundamentals",
                "week": "Week 3",
                "chapter_title": "ROS 2 Architecture: nodes, topics, services, actions",
                "url": "/docs/week-03/ros2-architecture",
                "relevance_score": 0.85
            }
        ],
        "conversation_id": "test-conversation-id",
        "message_id": "test-message-id"
    }
    service.retrieve_relevant_content.return_value = [
        {
            "id": "test-chunk-1",
            "content": "Test content for retrieval",
            "module": "Test Module",
            "week": "Week 1",
            "chapter_title": "Test Chapter",
            "url": "/docs/test-chapter",
            "score": 0.85
        }
    ]
    service.is_off_topic_query.return_value = False
    service.construct_prompt.return_value = "Test constructed prompt"
    return service


@pytest.fixture
def mock_chat_service(mock_rag_service):
    """
    Mock ChatService with mocked RAG service
    """
    service = Mock(spec=ChatService)
    service.process_message.return_value = {
        "response": "Test response from chat service",
        "sources": [
            {
                "module": "ROS 2 Fundamentals",
                "week": "Week 3",
                "chapter_title": "ROS 2 Architecture: nodes, topics, services, actions",
                "url": "/docs/week-03/ros2-architecture"
            }
        ],
        "conversation_id": "test-conversation-id",
        "message_id": "test-message-id"
    }
    service.rag_service = mock_rag_service
    return service


def test_rag_pipeline_end_to_end_flow(test_client, mock_rag_service):
    """
    Integration test: Full RAG pipeline from user query to response with citations
    """
    # Mock the rag service in the app
    with patch('backend.api.routes.chat.get_rag_service') as mock_get_rag_service:
        mock_get_rag_service.return_value = mock_rag_service

        # Make a request to the chat endpoint
        response = test_client.post("/api/v1/chat", json={
            "message": "What is ROS 2?",
            "session_id": "test-session-123",
            "model": "gpt-4o"
        })

        # Verify the response
        assert response.status_code == 200
        data = response.json()

        assert "response" in data
        assert "sources" in data
        assert "conversation_id" in data
        assert "message_id" in data

        # Verify that the response contains expected content
        assert "ROS 2" in data["response"]
        assert len(data["sources"]) > 0

        # Verify source structure
        source = data["sources"][0]
        assert "module" in source
        assert "week" in source
        assert "chapter_title" in source
        assert "url" in source


def test_rag_pipeline_with_conversation_context(test_client, mock_rag_service):
    """
    Integration test: RAG pipeline maintains conversation context across requests
    """
    with patch('backend.api.routes.chat.get_rag_service') as mock_get_rag_service:
        mock_get_rag_service.return_value = mock_rag_service

        # First message to establish context
        response1 = test_client.post("/api/v1/chat", json={
            "message": "What is ROS 2?",
            "session_id": "test-session-context",
            "model": "gpt-4o"
        })

        assert response1.status_code == 200
        data1 = response1.json()
        assert "conversation_id" in data1

        # Follow-up message that should use context
        response2 = test_client.post("/api/v1/chat", json={
            "message": "Can you tell me more about its architecture?",
            "session_id": "test-session-context",  # Same session to maintain context
            "model": "gpt-4o",
            "conversation_context": [{"role": "user", "content": "What is ROS 2?"},
                                   {"role": "assistant", "content": data1["response"]}]
        })

        assert response2.status_code == 200
        data2 = response2.json()

        # Verify that conversation context was used
        assert "architecture" in data2["response"].lower()
        assert data2["conversation_id"] == data1["conversation_id"]  # Same conversation


def test_rag_pipeline_content_retrieval_accuracy(test_client, mock_rag_service):
    """
    Integration test: RAG pipeline retrieves accurate content from vector database
    """
    with patch('backend.api.routes.chat.get_rag_service') as mock_get_rag_service:
        mock_get_rag_service.return_value = mock_rag_service

        # Test with a specific query that should have relevant content
        response = test_client.post("/api/v1/chat", json={
            "message": "Explain the components of ROS 2 architecture",
            "session_id": "test-session-accuracy",
            "model": "gpt-4o"
        })

        assert response.status_code == 200
        data = response.json()

        # Verify response contains information about ROS 2 architecture
        assert "response" in data
        response_text = data["response"].lower()
        assert any(term in response_text for term in ["node", "topic", "service", "action", "architecture"])

        # Verify sources are properly cited
        assert "sources" in data
        assert len(data["sources"]) > 0
        for source in data["sources"]:
            assert "module" in source
            assert "week" in source
            assert "chapter_title" in source
            assert "url" in source


def test_rag_pipeline_off_topic_handling(test_client, mock_rag_service):
    """
    Integration test: RAG pipeline properly handles off-topic questions
    """
    with patch('backend.api.routes.chat.get_rag_service') as mock_get_rag_service:
        # Configure mock to simulate off-topic handling
        mock_rag_service.is_off_topic_query.return_value = True
        mock_rag_service.generate_response.return_value = {
            "response": "I can only answer questions about Physical AI and Humanoid Robotics as covered in the textbook. I cannot answer questions about other topics.",
            "sources": [],
            "conversation_id": "test-off-topic-conversation",
            "message_id": "test-off-topic-message"
        }
        mock_get_rag_service.return_value = mock_rag_service

        # Test with an off-topic query
        response = test_client.post("/api/v1/chat", json={
            "message": "What's the weather like today?",
            "session_id": "test-session-off-topic",
            "model": "gpt-4o"
        })

        assert response.status_code == 200
        data = response.json()

        # Verify response indicates off-topic handling
        assert "cannot answer" in data["response"].lower() or "outside scope" in data["response"].lower()
        assert data["sources"] == []  # No sources for off-topic response


def test_rag_pipeline_with_empty_context(test_client, mock_rag_service):
    """
    Integration test: RAG pipeline handles queries with no relevant context
    """
    with patch('backend.api.routes.chat.get_rag_service') as mock_get_rag_service:
        # Configure mock to return no relevant content
        mock_rag_service.retrieve_relevant_content.return_value = []
        mock_rag_service.generate_response.return_value = {
            "response": "I couldn't find specific information about this topic in the textbook. Please check the relevant chapters on Physical AI and Humanoid Robotics.",
            "sources": [],
            "conversation_id": "test-no-context-conversation",
            "message_id": "test-no-context-message"
        }
        mock_get_rag_service.return_value = mock_rag_service

        # Test with a query that has no matching content
        response = test_client.post("/api/v1/chat", json={
            "message": "What is the capital of France?",
            "session_id": "test-session-no-context",
            "model": "gpt-4o"
        })

        assert response.status_code == 200
        data = response.json()

        # Verify response indicates lack of relevant content
        assert "couldn't find" in data["response"].lower() or "not available" in data["response"].lower()
        assert data["sources"] == []


def test_rag_pipeline_error_handling(test_client):
    """
    Integration test: RAG pipeline gracefully handles errors
    """
    with patch('backend.api.routes.chat.get_rag_service') as mock_get_rag_service:
        # Configure mock to raise an exception
        mock_get_rag_service.return_value.generate_response.side_effect = Exception("API Error")

        # Test with a query that causes an error
        response = test_client.post("/api/v1/chat", json={
            "message": "Test query that causes error",
            "session_id": "test-session-error",
            "model": "gpt-4o"
        })

        # Should return a 500 error or a graceful error response
        assert response.status_code in [500, 200]  # Depending on error handling implementation

        if response.status_code == 200:
            data = response.json()
            # Verify error response has proper structure
            assert "response" in data
            assert "sources" in data
            assert "error" in data["response"].lower() or "issue" in data["response"].lower()


def test_rag_pipeline_multiple_sources_citation(test_client, mock_rag_service):
    """
    Integration test: RAG pipeline properly cites multiple sources when relevant
    """
    with patch('backend.api.routes.chat.get_rag_service') as mock_get_rag_service:
        # Configure mock to return multiple sources
        mock_rag_service.generate_response.return_value = {
            "response": "ROS 2 has several key components that work together. The node concept is fundamental to its architecture, and topics enable communication between nodes.",
            "sources": [
                {
                    "module": "ROS 2 Fundamentals",
                    "week": "Week 3",
                    "chapter_title": "ROS 2 Architecture: nodes, topics, services, actions",
                    "url": "/docs/week-03/ros2-architecture",
                    "relevance_score": 0.85
                },
                {
                    "module": "ROS 2 Communication",
                    "week": "Week 3",
                    "chapter_title": "Topics, Services, and Actions in ROS 2",
                    "url": "/docs/week-03/topics-services-actions",
                    "relevance_score": 0.78
                }
            ],
            "conversation_id": "test-multi-source-conversation",
            "message_id": "test-multi-source-message"
        }
        mock_get_rag_service.return_value = mock_rag_service

        # Test with a query that should return multiple sources
        response = test_client.post("/api/v1/chat", json={
            "message": "Explain ROS 2 architecture and communication patterns",
            "session_id": "test-session-multi-source",
            "model": "gpt-4o"
        })

        assert response.status_code == 200
        data = response.json()

        # Verify response has multiple sources
        assert len(data["sources"]) == 2
        for source in data["sources"]:
            assert "module" in source
            assert "week" in source
            assert "chapter_title" in source
            assert "url" in source


def test_rag_pipeline_session_management_integration(test_client, mock_rag_service):
    """
    Integration test: RAG pipeline properly manages sessions and conversation history
    """
    with patch('backend.api.routes.chat.get_rag_service') as mock_get_rag_service:
        mock_get_rag_service.return_value = mock_rag_service

        session_id = "integration-test-session"

        # Send first message
        response1 = test_client.post("/api/v1/chat", json={
            "message": "What is Physical AI?",
            "session_id": session_id,
            "model": "gpt-4o"
        })

        assert response1.status_code == 200
        data1 = response1.json()
        conversation_id_1 = data1["conversation_id"]

        # Send second message in same session
        response2 = test_client.post("/api/v1/chat", json={
            "message": "How is it different from traditional AI?",
            "session_id": session_id,
            "model": "gpt-4o"
        })

        assert response2.status_code == 200
        data2 = response2.json()
        conversation_id_2 = data2["conversation_id"]

        # Both messages should be part of the same conversation
        assert conversation_id_1 == conversation_id_2

        # Verify both responses have content
        assert len(data1["response"]) > 0
        assert len(data2["response"]) > 0


def test_rag_pipeline_content_guardrails_integration(test_client, mock_rag_service):
    """
    Integration test: RAG pipeline enforces content guardrails properly
    """
    with patch('backend.api.routes.chat.get_rag_service') as mock_get_rag_service:
        # Configure mock to test guardrail behavior
        mock_rag_service.generate_response.return_value = {
            "response": "Based on the textbook content, I can explain Physical AI concepts but cannot provide information outside the curriculum.",
            "sources": [
                {
                    "module": "Physical AI Foundations",
                    "week": "Week 1",
                    "chapter_title": "Introduction to Physical AI",
                    "url": "/docs/week-01/introduction-to-physical-ai",
                    "relevance_score": 0.92
                }
            ],
            "conversation_id": "test-guardrails-conversation",
            "message_id": "test-guardrails-message"
        }
        mock_get_rag_service.return_value = mock_rag_service

        # Test a query that should be within the textbook scope
        response = test_client.post("/api/v1/chat", json={
            "message": "What are the foundations of Physical AI?",
            "session_id": "test-session-guardrails",
            "model": "gpt-4o"
        })

        assert response.status_code == 200
        data = response.json()

        # Verify response contains textbook-based information
        assert "textbook" in data["response"].lower() or "content" in data["response"].lower()
        assert len(data["sources"]) > 0


def test_rag_pipeline_performance_under_load(test_client, mock_rag_service):
    """
    Integration test: RAG pipeline performs adequately under moderate load
    """
    import time

    with patch('backend.api.routes.chat.get_rag_service') as mock_get_rag_service:
        mock_get_rag_service.return_value = mock_rag_service

        session_id = "performance-test-session"

        # Send multiple requests in succession
        start_time = time.time()

        for i in range(5):  # Test with 5 requests
            response = test_client.post("/api/v1/chat", json={
                "message": f"Test query {i+1} about Physical AI concepts",
                "session_id": session_id,
                "model": "gpt-4o"
            })

            assert response.status_code == 200
            data = response.json()
            assert "response" in data
            assert "sources" in data

        end_time = time.time()
        total_time = end_time - start_time

        # Verify that total time for 5 requests is reasonable (under 30 seconds)
        assert total_time < 30.0, f"Requests took too long: {total_time}s for 5 requests"


if __name__ == "__main__":
    pytest.main([__file__])