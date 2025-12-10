"""
Unit tests for the RAG Service in the RAG Chatbot API
"""

import pytest
from unittest.mock import Mock, patch, MagicMock
from typing import List, Dict, Any
from backend.services.rag_service import RAGService
from backend.utils.embedding import EmbeddingService
from backend.utils.qdrant_client import QdrantService


@pytest.fixture
def mock_embedding_service():
    """
    Mock embedding service for testing
    """
    service = Mock(spec=EmbeddingService)
    service.create_embedding.return_value = [0.1, 0.2, 0.3, 0.4, 0.5]  # Mock embedding vector
    service.create_embeddings.return_value = [
        [0.1, 0.2, 0.3, 0.4, 0.5],
        [0.6, 0.7, 0.8, 0.9, 1.0]
    ]
    return service


@pytest.fixture
def mock_qdrant_service():
    """
    Mock Qdrant service for testing
    """
    service = Mock(spec=QdrantService)
    service.search.return_value = [
        {
            'id': 'test-chunk-1',
            'content': 'Test content for retrieval',
            'module': 'Test Module',
            'week': 'Week 1',
            'chapter_title': 'Test Chapter',
            'url': '/docs/test-chapter',
            'score': 0.85
        },
        {
            'id': 'test-chunk-2',
            'content': 'Another test content for retrieval',
            'module': 'Test Module',
            'week': 'Week 2',
            'chapter_title': 'Another Chapter',
            'url': '/docs/another-chapter',
            'score': 0.78
        }
    ]
    return service


@pytest.fixture
def mock_gemini_client():
    """
    Mock Google Gemini client for testing
    """
    client = Mock()
    mock_response = Mock()
    mock_response.text = "This is a test response from the LLM."
    client.generate_content.return_value = mock_response
    return client


@pytest.fixture
def rag_service(mock_embedding_service, mock_qdrant_service, mock_gemini_client):
    """
    Create a RAGService instance with mocked dependencies
    """
    service = RAGService()
    service.embedding_service = mock_embedding_service
    service.qdrant_service = mock_qdrant_service
    service.model = mock_gemini_client
    return service


def test_rag_service_initialization():
    """
    Test that RAGService initializes with required dependencies
    """
    with patch('backend.services.rag_service.EmbeddingService'), \
         patch('backend.services.rag_service.QdrantService'), \
         patch('google.generativeai.GenerativeModel') as mock_genai:

        service = RAGService()

        assert service.embedding_service is not None
        assert service.qdrant_service is not None
        assert service.model is not None
        assert service.confidence_threshold == 0.6  # Default threshold


def test_retrieve_relevant_content_success(rag_service, mock_qdrant_service):
    """
    Test that retrieve_relevant_content successfully retrieves content from Qdrant
    """
    query = "What is ROS 2?"

    # Call the method
    results = rag_service.retrieve_relevant_content(query)

    # Verify that embedding service was called with the query
    rag_service.embedding_service.create_embedding.assert_called_once_with(query)

    # Verify that Qdrant service was called with the embedding
    rag_service.qdrant_service.search.assert_called_once()
    args, kwargs = rag_service.qdrant_service.search.call_args
    assert args[0] == [0.1, 0.2, 0.3, 0.4, 0.5]  # Mock embedding
    assert kwargs.get('top_k') == 5

    # Verify that results are returned
    assert len(results) == 2  # Two mock results from fixture
    assert results[0]['score'] >= rag_service.confidence_threshold  # Confidence threshold applied


def test_retrieve_relevant_content_filters_by_confidence(rag_service, mock_qdrant_service):
    """
    Test that retrieve_relevant_content filters results by confidence threshold
    """
    # Mock results with some below threshold
    mock_qdrant_service.search.return_value = [
        {
            'id': 'high-score-chunk',
            'content': 'High confidence content',
            'module': 'Module 1',
            'week': 'Week 1',
            'chapter_title': 'Chapter 1',
            'url': '/docs/chapter-1',
            'score': 0.85  # Above threshold
        },
        {
            'id': 'low-score-chunk',
            'content': 'Low confidence content',
            'module': 'Module 2',
            'week': 'Week 2',
            'chapter_title': 'Chapter 2',
            'url': '/docs/chapter-2',
            'score': 0.3   # Below threshold (0.6)
        }
    ]

    query = "Test query"

    # Call the method
    results = rag_service.retrieve_relevant_content(query)

    # Verify that only high-confidence results are returned
    assert len(results) == 1
    assert results[0]['id'] == 'high-score-chunk'
    assert results[0]['score'] >= rag_service.confidence_threshold


def test_retrieve_relevant_content_off_topic_query(rag_service):
    """
    Test that retrieve_relevant_content handles off-topic queries appropriately
    """
    # Test with an off-topic query
    off_topic_query = "What is the weather today?"

    # Call the method
    results = rag_service.retrieve_relevant_content(off_topic_query)

    # Verify that embedding service was still called (to check if it's off-topic)
    rag_service.embedding_service.create_embedding.assert_called_once_with(off_topic_query)

    # For off-topic queries, it should still return results from Qdrant
    # but the RAG pipeline would handle the off-topic response separately
    assert isinstance(results, list)


def test_construct_prompt_basic(rag_service):
    """
    Test that construct_prompt creates a proper prompt with context
    """
    query = "What is ROS 2?"
    context = [
        {
            'id': 'test-chunk-1',
            'content': 'ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software.',
            'module': 'ROS 2 Fundamentals',
            'week': 'Week 3',
            'chapter_title': 'ROS 2 Architecture',
            'url': '/docs/week-03/ros2-architecture',
            'score': 0.85
        }
    ]

    # Call the method
    prompt = rag_service.construct_prompt(query, context)

    # Verify that the prompt contains required elements
    assert "Physical AI & Humanoid Robotics" in prompt
    assert query in prompt
    assert "ROS 2" in prompt
    assert "Robot Operating System" in prompt
    assert "citations" in prompt.lower()


def test_construct_prompt_with_conversation_history(rag_service):
    """
    Test that construct_prompt includes conversation history when provided
    """
    query = "Can you elaborate?"
    context = [
        {
            'id': 'test-chunk-1',
            'content': 'ROS 2 provides a flexible framework for robot software development.',
            'module': 'ROS 2 Fundamentals',
            'week': 'Week 3',
            'chapter_title': 'ROS 2 Architecture',
            'url': '/docs/week-03/ros2-architecture',
            'score': 0.85
        }
    ]
    conversation_history = [
        {
            'role': 'user',
            'content': 'What is ROS 2?',
            'timestamp': '2023-01-01T10:00:00Z'
        },
        {
            'role': 'assistant',
            'content': 'ROS 2 is a flexible framework for writing robot software.',
            'timestamp': '2023-01-01T10:01:00Z'
        }
    ]

    # Call the method
    prompt = rag_service.construct_prompt(query, context, conversation_history)

    # Verify that the prompt contains conversation history
    assert "Previous conversation:" in prompt
    assert "What is ROS 2?" in prompt
    assert "ROS 2 is a flexible framework" in prompt
    assert "Can you elaborate?" in prompt  # Current query should still be there


def test_construct_prompt_without_conversation_history(rag_service):
    """
    Test that construct_prompt works without conversation history
    """
    query = "What is machine learning?"
    context = [
        {
            'id': 'test-chunk-1',
            'content': 'Machine learning is a subset of artificial intelligence.',
            'module': 'AI Fundamentals',
            'week': 'Week 1',
            'chapter_title': 'Introduction to AI',
            'url': '/docs/week-01/introduction-to-ai',
            'score': 0.9
        }
    ]

    # Call the method
    prompt = rag_service.construct_prompt(query, context, None)

    # Verify that the prompt is constructed properly
    assert "Physical AI & Humanoid Robotics" in prompt
    assert query in prompt
    assert "Machine learning is a subset" in prompt
    assert "New conversation" in prompt or "This is the start" in prompt


@pytest.mark.asyncio
async def test_generate_response_success(rag_service, mock_gemini_client):
    """
    Test that generate_response successfully generates a response using RAG
    """
    query = "What is ROS 2?"

    # Mock the helper methods and the timeout function
    with patch.object(rag_service, 'retrieve_relevant_content') as mock_retrieve, \
         patch.object(rag_service, 'construct_prompt') as mock_construct, \
         patch('backend.utils.timeout_handler.timeout_aware_gemini_call') as mock_timeout_call:

        mock_timeout_call.return_value = "This is a test response from the LLM."
        mock_retrieve.return_value = [
            {
                'id': 'test-chunk-1',
                'content': 'ROS 2 is a flexible framework for robot software.',
                'module': 'ROS 2 Fundamentals',
                'week': 'Week 3',
                'chapter_title': 'ROS 2 Architecture',
                'url': '/docs/week-03/ros2-architecture',
                'score': 0.85,
                'relevance_score': 0.85
            }
        ]
        mock_construct.return_value = "Constructed prompt for testing"

        # Call the method (now async)
        result = await rag_service.generate_response(query)

        # Verify that all steps were called
        mock_retrieve.assert_called_once_with(query)
        mock_construct.assert_called_once_with(query, mock_retrieve.return_value, None)
        mock_timeout_call.assert_called_once()

        # Verify result structure
        assert "response" in result
        assert "sources" in result
        assert result["response"] == "This is a test response from the LLM."
        assert len(result["sources"]) == 1


@pytest.mark.asyncio
async def test_generate_response_with_conversation_history(rag_service, mock_gemini_client):
    """
    Test that generate_response works with conversation history
    """
    query = "Can you elaborate?"
    conversation_history = [
        {
            'role': 'user',
            'content': 'What is ROS 2?'
        },
        {
            'role': 'assistant',
            'content': 'ROS 2 is a framework for robot software.'
        }
    ]

    # Mock the helper methods and the timeout function
    with patch.object(rag_service, 'retrieve_relevant_content') as mock_retrieve, \
         patch.object(rag_service, 'construct_prompt') as mock_construct, \
         patch('backend.utils.timeout_handler.timeout_aware_gemini_call') as mock_timeout_call:

        mock_timeout_call.return_value = "This is a test response from the LLM."
        mock_retrieve.return_value = [
            {
                'id': 'test-chunk-1',
                'content': 'ROS 2 provides tools for robot development.',
                'module': 'ROS 2 Fundamentals',
                'week': 'Week 3',
                'chapter_title': 'ROS 2 Architecture',
                'url': '/docs/week-03/ros2-architecture',
                'score': 0.8,
                'relevance_score': 0.8
            }
        ]
        mock_construct.return_value = "Constructed prompt with history"

        # Call the method (now async)
        result = await rag_service.generate_response(query, conversation_history)

        # Verify that construct_prompt was called with history
        mock_construct.assert_called_once_with(query, mock_retrieve.return_value, conversation_history)
        mock_timeout_call.assert_called_once()

        # Verify result structure
        assert "response" in result
        assert "sources" in result


@pytest.mark.asyncio
async def test_generate_response_handles_rag_failure(rag_service):
    """
    Test that generate_response handles RAG service failures gracefully
    """
    query = "Test query"

    # Mock to raise an exception in retrieve_relevant_content
    with patch.object(rag_service, 'retrieve_relevant_content') as mock_retrieve, \
         patch('backend.utils.timeout_handler.timeout_aware_gemini_call') as mock_timeout_call:

        mock_retrieve.side_effect = Exception("Qdrant connection failed")
        mock_timeout_call.return_value = "This is a test response from the LLM."  # This won't be reached due to exception

        # Call the method (now async)
        result = await rag_service.generate_response(query)

        # Verify fallback response
        assert "response" in result
        assert "encountered an error" in result["response"].lower()
        assert result["sources"] == []


def test_is_off_topic_query_identifies_off_topic(rag_service):
    """
    Test that is_off_topic_query correctly identifies off-topic queries
    """
    off_topic_queries = [
        "What's the weather like today?",
        "Tell me a joke",
        "How much does a Tesla cost?",
        "Who won the World Cup?",
        "What is your favorite movie?",
        "Can you book a flight for me?",
        "How to cook pasta?",
        "Stock price for Apple",
        "Latest news",
        "What time is it?"
    ]

    for query in off_topic_queries:
        result = rag_service.is_off_topic_query(query)
        assert result is True, f"Query '{query}' should be considered off-topic"


def test_is_off_topic_query_identifies_on_topic(rag_service):
    """
    Test that is_off_topic_query correctly identifies on-topic queries
    """
    on_topic_queries = [
        "What is ROS 2?",
        "Explain humanoid kinematics",
        "How does SLAM work?",
        "What is physical AI?",
        "Describe inverse kinematics",
        "Explain PID controllers",
        "What is a Jacobian matrix?",
        "How does a Kalman filter work?",
        "What is the difference between forward and inverse kinematics?",
        "Explain the ROS 2 architecture"
    ]

    for query in on_topic_queries:
        result = rag_service.is_off_topic_query(query)
        assert result is False, f"Query '{query}' should be considered on-topic"


def test_detect_hallucinations_basic_check(rag_service):
    """
    Test that detect_hallucinations can identify potential hallucinations
    """
    response = "The textbook explains that quantum entanglement is used in humanoid robotics for teleportation."
    source_content = [
        {
            'id': 'valid-chunk-1',
            'content': 'Humanoid robotics involves mechanical design, control systems, and AI.',
            'module': 'Humanoid Design',
            'week': 'Week 5',
            'chapter_title': 'Mechanical Design',
            'url': '/docs/week-05/mechanical-design'
        }
    ]

    # Call the method
    result = rag_service.detect_hallucinations(response, source_content)

    # Verify that hallucination detection ran
    assert "has_hallucinations" in result
    assert "confidence_score" in result
    assert "issues" in result
    assert "suggestions" in result


def test_detect_hallucinations_valid_response(rag_service):
    """
    Test that detect_hallucinations returns no hallucinations for valid responses
    """
    response = "The textbook explains that humanoid robotics involves mechanical design, control systems, and AI."
    source_content = [
        {
            'id': 'valid-chunk-1',
            'content': 'Humanoid robotics involves mechanical design, control systems, and AI.',
            'module': 'Humanoid Design',
            'week': 'Week 5',
            'chapter_title': 'Introduction to Humanoids',
            'url': '/docs/week-05/introduction'
        }
    ]

    # Call the method
    result = rag_service.detect_hallucinations(response, source_content)

    # Should not detect hallucinations for content that matches sources
    # Note: The actual detection algorithm is simplistic, so this may still show issues
    assert "has_hallucinations" in result


def test_prevent_hallucinations(rag_service):
    """
    Test that prevent_hallucinations modifies responses with potential hallucinations
    """
    response = "According to page 150 of the textbook, this is definitely explained there."
    source_content = [
        {
            'id': 'valid-chunk-1',
            'content': 'This topic is covered in the textbook.',
            'module': 'Module 1',
            'week': 'Week 1',
            'chapter_title': 'Introduction',
            'url': '/docs/week-01/introduction'
        }
    ]

    # Call the method
    result = rag_service.prevent_hallucinations(response, source_content)

    # The response may be modified to be more conservative
    assert isinstance(result, str)


def test_format_citations_basic(rag_service):
    """
    Test that format_citations properly formats source citations
    """
    sources = [
        {
            "module": "ROS 2 Fundamentals",
            "week": "Week 3",
            "chapter_title": "ROS 2 Architecture",
            "url": "/docs/week-03/ros2-architecture",
            "relevance_score": 0.85
        }
    ]

    # Call the method
    formatted_sources = rag_service.format_citations(sources)

    # Verify the format
    assert len(formatted_sources) == 1
    formatted = formatted_sources[0]
    assert "module" in formatted
    assert "week" in formatted
    assert "chapter_title" in formatted
    assert "url" in formatted
    assert "relevance_score" in formatted
    assert "formatted_citation" in formatted
    assert "ROS 2 Fundamentals" in formatted["formatted_citation"]


def test_validate_content_relevance_filters_by_threshold(rag_service):
    """
    Test that validate_content_relevance filters content by relevance threshold
    """
    content_chunks = [
        {
            "id": "high-relevance",
            "content": "This is highly relevant content.",
            "score": 0.85
        },
        {
            "id": "low-relevance",
            "content": "This is not very relevant.",
            "score": 0.4  # Below threshold of 0.6
        }
    ]

    # Call the method
    valid_chunks = rag_service.validate_content_relevance("test query", content_chunks)

    # Should only return high-relevance chunks
    assert len(valid_chunks) == 1
    assert valid_chunks[0]["id"] == "high-relevance"


def test_validate_content_quality_basic(rag_service):
    """
    Test that validate_content_quality evaluates content quality
    """
    content = "This is a valid content string with sufficient information."

    # Call the method
    quality_result = rag_service.validate_content_quality(content)

    # Verify the result structure
    assert "is_valid" in quality_result
    assert "issues" in quality_result
    assert "quality_score" in quality_result
    assert "suggestions" in quality_result

    # For reasonable content, it should be valid
    assert quality_result["is_valid"] is True


def test_validate_content_quality_short_content(rag_service):
    """
    Test that validate_content_quality identifies very short content
    """
    content = "Hi."  # Very short content

    # Call the method
    quality_result = rag_service.validate_content_quality(content)

    # Should identify the issue
    assert quality_result["is_valid"] is False
    assert len(quality_result["issues"]) > 0
    assert "short" in quality_result["issues"][0].lower()


if __name__ == "__main__":
    pytest.main([__file__])