from typing import Optional
from sqlalchemy.orm import Session
from backend.models.conversation import Conversation
from backend.utils.database import get_db_sync
from datetime import datetime, timezone
import uuid

def get_or_create_session(session_id: str, db: Session) -> Conversation:
    """
    Get an existing session or create a new one if it doesn't exist
    """
    # Try to find existing conversation with this session_id
    conversation = db.query(Conversation).filter(
        Conversation.session_id == session_id
    ).first()

    if conversation:
        # Update the last activity time
        conversation.updated_at = datetime.now(timezone.utc)
        db.commit()
        return conversation
    else:
        # Create a new conversation
        conversation = Conversation(
            session_id=session_id,
            created_at=datetime.now(timezone.utc),
            updated_at=datetime.now(timezone.utc)
        )
        db.add(conversation)
        db.commit()
        db.refresh(conversation)
        return conversation

def create_new_session() -> str:
    """
    Create a new unique session ID
    """
    return str(uuid.uuid4())

def get_conversation_by_session(session_id: str, db: Session) -> Optional[Conversation]:
    """
    Get a conversation by session ID
    """
    return db.query(Conversation).filter(
        Conversation.session_id == session_id
    ).first()

def get_conversation_history(conversation_id: str, db: Session, limit: int = 10):
    """
    Get the message history for a conversation
    """
    from backend.models.message import Message

    return db.query(Message).filter(
        Message.conversation_id == conversation_id
    ).order_by(Message.timestamp.desc()).limit(limit).all()

def cleanup_old_sessions(db: Session, days_old: int = 30):
    """
    Clean up sessions that are older than specified days
    """
    from datetime import datetime, timedelta
    from backend.models.message import Message

    cutoff_date = datetime.now(timezone.utc) - timedelta(days=days_old)

    # Find conversations older than cutoff date
    old_conversations = db.query(Conversation).filter(
        Conversation.updated_at < cutoff_date
    ).all()

    # Delete old conversations (and their messages due to CASCADE)
    for conversation in old_conversations:
        db.delete(conversation)

    db.commit()

def validate_session(session_id: str) -> bool:
    """
    Validate if a session ID is properly formatted
    """
    try:
        uuid.UUID(session_id)
        return True
    except ValueError:
        # If it's not a UUID, it might be a custom session ID format
        # For now, we'll accept any non-empty string as valid
        return bool(session_id and len(session_id.strip()) > 0)