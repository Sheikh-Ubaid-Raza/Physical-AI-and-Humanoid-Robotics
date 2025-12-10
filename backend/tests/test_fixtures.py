"""
Test data fixtures for the RAG Chatbot API
"""

from datetime import datetime, timezone
from typing import Dict, List, Any
import uuid
from backend.models.conversation import Conversation
from backend.models.message import Message


class TestFixtures:
    """
    Test data fixtures for consistent testing
    """

    @staticmethod
    def get_sample_conversation() -> Conversation:
        """
        Get a sample conversation fixture
        """
        return Conversation(
            id=uuid.uuid4(),
            session_id="test-session-123",
            created_at=datetime.now(timezone.utc),
            updated_at=datetime.now(timezone.utc)
        )

    @staticmethod
    def get_sample_message(role: str = "user", content: str = "Test message content") -> Message:
        """
        Get a sample message fixture
        """
        return Message(
            id=uuid.uuid4(),
            conversation_id=uuid.uuid4(),
            role=role,
            content=content,
            sources=[],
            timestamp=datetime.utcnow()
        )

    @staticmethod
    def get_sample_user_message(content: str = "What is ROS 2?") -> Message:
        """
        Get a sample user message fixture
        """
        return Message(
            id=uuid.uuid4(),
            conversation_id=uuid.uuid4(),
            role="user",
            content=content,
            sources=[],
            timestamp=datetime.utcnow()
        )

    @staticmethod
    def get_sample_assistant_message(content: str = "ROS 2 is a flexible framework for writing robot software.") -> Message:
        """
        Get a sample assistant message fixture
        """
        return Message(
            id=uuid.uuid4(),
            conversation_id=uuid.uuid4(),
            role="assistant",
            content=content,
            sources=[
                {
                    "module": "ROS 2 Fundamentals",
                    "week": "Week 3",
                    "chapter_title": "ROS 2 Architecture: nodes, topics, services, actions",
                    "url": "/docs/week-03/ros2-architecture"
                }
            ],
            timestamp=datetime.utcnow()
        )

    @staticmethod
    def get_sample_chat_request(session_id: str = "test-session-456") -> Dict[str, Any]:
        """
        Get a sample chat request fixture
        """
        return {
            "message": "What is Physical AI?",
            "session_id": session_id,
            "model": "gpt-4o"
        }

    @staticmethod
    def get_sample_chat_request_with_context(session_id: str = "test-session-789") -> Dict[str, Any]:
        """
        Get a sample chat request with conversation context
        """
        return {
            "message": "Can you elaborate on that?",
            "session_id": session_id,
            "model": "gpt-4o",
            "conversation_context": [
                {
                    "role": "user",
                    "content": "What is Physical AI?"
                },
                {
                    "role": "assistant",
                    "content": "Physical AI combines artificial intelligence with physical systems."
                }
            ]
        }

    @staticmethod
    def get_sample_ingest_request() -> Dict[str, Any]:
        """
        Get a sample ingestion request fixture
        """
        return {
            "module": "Week 1",
            "week": "Week 1",
            "chapter_title": "Foundations of Physical AI",
            "content": "Physical AI combines artificial intelligence with physical systems to create intelligent robots and embodied agents. This field encompasses various aspects including sensor systems, actuation, control theory, and machine learning applications in physical environments.",
            "url": "/docs/week-01/foundations-of-physical-ai"
        }

    @staticmethod
    def get_sample_rag_response() -> Dict[str, Any]:
        """
        Get a sample RAG response fixture
        """
        return {
            "response": "Physical AI combines artificial intelligence with physical systems to create intelligent robots and embodied agents. This field encompasses various aspects including sensor systems, actuation, control theory, and machine learning applications in physical environments.",
            "sources": [
                {
                    "module": "Physical AI Foundations",
                    "week": "Week 1",
                    "chapter_title": "Foundations of Physical AI",
                    "url": "/docs/week-01/foundations-of-physical-ai",
                    "relevance_score": 0.92
                }
            ],
            "conversation_id": "test-conversation-id",
            "message_id": "test-message-id"
        }

    @staticmethod
    def get_sample_qdrant_payload() -> Dict[str, Any]:
        """
        Get a sample Qdrant payload fixture
        """
        return {
            "module": "ROS 2 Fundamentals",
            "week": "Week 3",
            "chapter_title": "ROS 2 Architecture: nodes, topics, services, actions",
            "content": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.",
            "url": "/docs/week-03/ros2-architecture",
            "embedding": [0.1, 0.2, 0.3, 0.4, 0.5]  # Simplified embedding for testing
        }

    @staticmethod
    def get_sample_textbook_content() -> List[Dict[str, Any]]:
        """
        Get sample textbook content fixtures for testing the ingestion pipeline
        """
        return [
            {
                "module": "Week 1",
                "week": "Week 1",
                "chapter_title": "Foundations of Physical AI",
                "content": "Physical AI combines artificial intelligence with physical systems to create intelligent robots and embodied agents. This field encompasses various aspects including sensor systems, actuation, control theory, and machine learning applications in physical environments. The core principle is that intelligence emerges from the interaction between computational algorithms and physical systems.",
                "url": "/docs/week-01/foundations-of-physical-ai"
            },
            {
                "module": "Week 2",
                "week": "Week 2",
                "chapter_title": "Sensor Systems in Physical AI",
                "content": "Sensor systems form the foundation of Physical AI by providing robots with awareness of their environment. Common sensors include LIDAR, cameras, IMUs, force/torque sensors, and tactile sensors. Each sensor type has specific characteristics and limitations that must be understood to properly integrate them into robotic systems.",
                "url": "/docs/week-02/sensor-systems"
            },
            {
                "module": "Week 3",
                "week": "Week 3",
                "chapter_title": "ROS 2 Architecture: nodes, topics, services, actions",
                "content": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. The core concepts include nodes, topics, services, and actions.",
                "url": "/docs/week-03/ros2-architecture"
            },
            {
                "module": "Week 5",
                "week": "Week 5",
                "chapter_title": "Humanoid Kinematics and Dynamics",
                "content": "Humanoid robots present unique challenges in terms of kinematics and dynamics. Unlike wheeled robots, humanoids must maintain balance while performing tasks. This requires understanding of forward and inverse kinematics, center of mass calculations, and dynamic stability principles.",
                "url": "/docs/week-05/humanoid-kinematics"
            }
        ]

    @staticmethod
    def get_sample_chat_history() -> List[Dict[str, Any]]:
        """
        Get sample chat history for testing conversation context
        """
        return [
            {
                "id": "msg-001",
                "role": "user",
                "content": "What is Physical AI?",
                "timestamp": "2025-12-08T10:00:00Z",
                "sources": []
            },
            {
                "id": "msg-002",
                "role": "assistant",
                "content": "Physical AI combines artificial intelligence with physical systems to create intelligent robots and embodied agents.",
                "timestamp": "2025-12-08T10:01:00Z",
                "sources": [
                    {
                        "module": "Physical AI Foundations",
                        "week": "Week 1",
                        "chapter_title": "Foundations of Physical AI",
                        "url": "/docs/week-01/foundations-of-physical-ai"
                    }
                ]
            },
            {
                "id": "msg-003",
                "role": "user",
                "content": "Can you tell me more about sensor systems?",
                "timestamp": "2025-12-08T10:02:00Z",
                "sources": []
            },
            {
                "id": "msg-004",
                "role": "assistant",
                "content": "Sensor systems are crucial for Physical AI. Common sensors include LIDAR, cameras, IMUs, force/torque sensors, and tactile sensors.",
                "timestamp": "2025-12-08T10:03:00Z",
                "sources": [
                    {
                        "module": "Sensor Systems",
                        "week": "Week 2",
                        "chapter_title": "Sensor Systems in Physical AI",
                        "url": "/docs/week-02/sensor-systems"
                    }
                ]
            }
        ]

    @staticmethod
    def get_sample_embedding() -> List[float]:
        """
        Get a sample embedding vector for testing
        """
        return [0.12, 0.45, 0.68, 0.23, 0.89, 0.34, 0.56, 0.78, 0.91, 0.05] + [0.0] * 1526  # Pad to 1536 dimensions

    @staticmethod
    def get_sample_chunked_content() -> List[Dict[str, Any]]:
        """
        Get sample chunked content for testing the chunking functionality
        """
        return [
            {
                "id": "chunk-001",
                "content": "Physical AI combines artificial intelligence with physical systems to create intelligent robots and embodied agents.",
                "module": "Week 1",
                "week": "Week 1",
                "chapter_title": "Foundations of Physical AI",
                "url": "/docs/week-01/foundations-of-physical-ai",
                "token_count": 25,
                "overlap_with_next": True
            },
            {
                "id": "chunk-002",
                "content": "This field encompasses various aspects including sensor systems, actuation, control theory, and machine learning applications in physical environments.",
                "module": "Week 1",
                "week": "Week 1",
                "chapter_title": "Foundations of Physical AI",
                "url": "/docs/week-01/foundations-of-physical-ai",
                "token_count": 30,
                "overlap_with_next": False
            }
        ]

    @staticmethod
    def get_test_configuration() -> Dict[str, Any]:
        """
        Get test configuration with API keys and settings for testing
        """
        return {
            "GEMINI_API_KEY": "test-gemini-key",
            "QDRANT_URL": "http://localhost:6333",  # Use local Qdrant for testing
            "QDRANT_API_KEY": "test-qdrant-key",
            "NEON_DATABASE_URL": "postgresql://test:test@test:5432/test",
            "MODEL_NAME": "gemini-2.5-flash",
            "EMBEDDING_MODEL": "embedding-001",
            "CONFIDENCE_THRESHOLD": 0.6,
            "MAX_TOKENS": 1000,
            "TEMPERATURE": 0.3
        }


# Create a global instance of test fixtures
test_fixtures = TestFixtures()


def get_test_fixtures() -> TestFixtures:
    """
    Get the global test fixtures instance
    """
    return test_fixtures


# Convenience functions for getting specific fixtures
def get_sample_conversation_fixture() -> Conversation:
    """Get a sample conversation fixture"""
    return test_fixtures.get_sample_conversation()


def get_sample_message_fixture(role: str = "user", content: str = "Test message content") -> Message:
    """Get a sample message fixture"""
    return test_fixtures.get_sample_message(role, content)


def get_sample_chat_request_fixture(session_id: str = "test-session-123") -> Dict[str, Any]:
    """Get a sample chat request fixture"""
    return test_fixtures.get_sample_chat_request(session_id)


def get_sample_textbook_content_fixture() -> List[Dict[str, Any]]:
    """Get sample textbook content fixtures"""
    return test_fixtures.get_sample_textbook_content()


def get_sample_chat_history_fixture() -> List[Dict[str, Any]]:
    """Get sample chat history fixture"""
    return test_fixtures.get_sample_chat_history()


def get_test_config_fixture() -> Dict[str, Any]:
    """Get test configuration fixture"""
    return test_fixtures.get_test_configuration()