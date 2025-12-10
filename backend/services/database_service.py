"""
Database service with parameterized queries for the RAG Chatbot API
"""

from sqlalchemy.orm import Session
from sqlalchemy import text
from typing import Optional, List
from backend.models.conversation import Conversation
from backend.models.message import Message
from datetime import datetime, timezone, timedelta
import uuid


class DatabaseService:
    """
    Service layer with parameterized queries to prevent SQL injection
    """
    def __init__(self, db_session: Session):
        self.db = db_session

    def get_or_create_conversation(self, session_id: str) -> Conversation:
        """
        Get existing conversation or create a new one using parameterized query
        """
        # Using SQLAlchemy ORM which inherently uses parameterized queries
        conversation = self.db.query(Conversation).filter(
            Conversation.session_id == session_id
        ).first()

        if not conversation:
            conversation = Conversation(
                id=uuid.uuid4(),
                session_id=session_id,
                created_at=datetime.now(timezone.utc)
            )
            self.db.add(conversation)
            self.db.commit()
            self.db.refresh(conversation)

        return conversation

    def create_message(self, conversation_id: uuid.UUID, role: str, content: str, sources: Optional[List[dict]] = None) -> Message:
        """
        Create a message using parameterized query
        """
        message = Message(
            id=uuid.uuid4(),
            conversation_id=conversation_id,
            role=role,
            content=content,
            sources=sources or [],
            timestamp=datetime.now(timezone.utc)
        )
        self.db.add(message)
        self.db.commit()
        self.db.refresh(message)
        return message

    def get_conversation_messages(self, conversation_id: uuid.UUID, limit: int = 50) -> List[Message]:
        """
        Get messages for a conversation using parameterized query
        """
        # Using SQLAlchemy ORM which provides parameterized queries
        messages = self.db.query(Message).filter(
            Message.conversation_id == conversation_id
        ).order_by(Message.timestamp.desc()).limit(limit).all()

        # Reverse to get chronological order (oldest first)
        return list(reversed(messages))

    def get_recent_conversations(self, session_id: str, limit: int = 10) -> List[Conversation]:
        """
        Get recent conversations for a session using parameterized query
        """
        conversations = self.db.query(Conversation).filter(
            Conversation.session_id == session_id
        ).order_by(Conversation.updated_at.desc()).limit(limit).all()

        return conversations

    def update_conversation_timestamp(self, conversation_id: uuid.UUID):
        """
        Update conversation timestamp using parameterized query
        """
        conversation = self.db.query(Conversation).filter(
            Conversation.id == conversation_id
        ).first()

        if conversation:
            conversation.updated_at = datetime.now(timezone.utc)
            self.db.commit()

    def delete_old_conversations(self, days_old: int = 30):
        """
        Delete conversations older than specified days using parameterized query
        """
        cutoff_date = datetime.now(timezone.utc) - timedelta(days=days_old)

        # Using SQLAlchemy's parameterized query mechanism
        old_conversations = self.db.query(Conversation).filter(
            Conversation.updated_at < cutoff_date
        ).all()

        for conversation in old_conversations:
            self.db.delete(conversation)

        self.db.commit()

    def search_messages_by_content(self, query: str, limit: int = 10) -> List[Message]:
        """
        Search messages by content using parameterized query to prevent SQL injection
        """
        # Using SQLAlchemy's parameterized query mechanism with ilike for case-insensitive search
        messages = self.db.query(Message).filter(
            Message.content.ilike(f'%{query}%')  # SQLAlchemy automatically parameterizes this
        ).limit(limit).all()

        return messages

    def get_conversation_stats(self, session_id: str) -> dict:
        """
        Get conversation statistics using parameterized queries
        """
        # Using raw SQL with parameterized queries for more complex aggregations
        result = self.db.execute(
            text("""
                SELECT
                    COUNT(*) as total_messages,
                    MIN(timestamp) as first_message,
                    MAX(timestamp) as last_message
                FROM messages m
                JOIN conversations c ON m.conversation_id = c.id
                WHERE c.session_id = :session_id
            """),
            {"session_id": session_id}
        ).fetchone()

        return {
            "total_messages": result[0] if result else 0,
            "first_message": result[1] if result else None,
            "last_message": result[2] if result else None
        }

    def get_messages_with_sources(self, conversation_id: uuid.UUID) -> List[Message]:
        """
        Get messages with sources using parameterized query
        """
        messages = self.db.query(Message).filter(
            Message.conversation_id == conversation_id,
            Message.sources.isnot(None)  # Only messages with sources
        ).order_by(Message.timestamp).all()

        return messages

    def bulk_insert_messages(self, messages_data: List[dict]) -> List[Message]:
        """
        Bulk insert messages using parameterized queries
        """
        messages = []
        for msg_data in messages_data:
            message = Message(
                id=uuid.uuid4(),
                conversation_id=msg_data['conversation_id'],
                role=msg_data['role'],
                content=msg_data['content'],
                sources=msg_data.get('sources', []),
                timestamp=msg_data.get('timestamp', datetime.now(timezone.utc))
            )
            self.db.add(message)
            messages.append(message)

        self.db.commit()
        return messages

    def validate_session_exists(self, session_id: str) -> bool:
        """
        Validate if a session exists using parameterized query
        """
        count = self.db.query(Conversation).filter(
            Conversation.session_id == session_id
        ).count()

        return count > 0


# Global instance
db_service = DatabaseService


def get_db_service(db_session: Session) -> DatabaseService:
    """
    Get database service instance with provided session
    """
    return DatabaseService(db_session)