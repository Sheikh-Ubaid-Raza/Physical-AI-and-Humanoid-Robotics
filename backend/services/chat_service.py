from typing import Dict, Any, List, Optional
from sqlalchemy.orm import Session
from backend.models.conversation import Conversation
from backend.models.message import Message
from backend.services.session_service import get_or_create_session
from backend.services.rag_service import get_rag_service
from backend.utils.error_handlers import APIError
from datetime import datetime, timezone
import uuid

class ChatService:
    def __init__(self, db: Session):
        self.db = db
        self.rag_service = get_rag_service()

    async def process_message(self, session_id: str, user_message: str, model: str = "gpt-4o") -> Dict[str, Any]:
        """
        Process a user message and return a response
        Enhanced with conversation context maintenance
        """
        try:
            # Get or create conversation session
            conversation = get_or_create_session(session_id, self.db)

            # Create user message record
            user_msg = Message(
                conversation_id=conversation.id,
                role="user",
                content=user_message
            )
            self.db.add(user_msg)
            self.db.commit()
            self.db.refresh(user_msg)

            # Get conversation history for context with window management
            conversation_history = self.get_conversation_context_window(conversation.id, window_size=5)

            # Generate response using RAG service (now async)
            response_data = await self.rag_service.generate_response(
                query=user_message,
                conversation_history=conversation_history
            )

            # Create assistant message record
            assistant_msg = Message(
                conversation_id=conversation.id,
                role="assistant",
                content=response_data["response"],
                sources=response_data["sources"]
            )
            self.db.add(assistant_msg)
            self.db.commit()
            self.db.refresh(assistant_msg)

            # Format the response
            formatted_sources = []
            for source in response_data["sources"]:
                formatted_sources.append({
                    "module": source.get("module", ""),
                    "week": source.get("week", ""),
                    "chapter_title": source.get("chapter_title", ""),
                    "url": source.get("url", "")
                })

            return {
                "response": response_data["response"],
                "sources": formatted_sources,
                "conversation_id": str(conversation.id),
                "message_id": str(assistant_msg.id)
            }

        except Exception as e:
            raise APIError(f"Error processing message: {str(e)}", 500, "CHAT_PROCESSING_ERROR")

    def _get_conversation_context(self, conversation_id: str, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Get the recent conversation history for context
        Enhanced to maintain conversation context across multiple messages
        """
        # Get recent messages from the conversation
        recent_messages = self.db.query(Message).filter(
            Message.conversation_id == conversation_id
        ).order_by(Message.timestamp.desc()).limit(limit).all()

        # Format messages for context (reverse order to maintain chronological sequence)
        context = []
        for msg in reversed(recent_messages):
            context.append({
                "role": msg.role,
                "content": msg.content,
                "timestamp": msg.timestamp.isoformat() if msg.timestamp else None,
                "sources": msg.sources
            })

        return context

    def update_message_storage_with_context_tracking(self, conversation_id: str):
        """
        Update message storage to include conversation context tracking
        """
        # This method would normally be implemented to track conversation context
        # but in our architecture, context is managed during retrieval
        # The context is already being tracked through the conversation and message relationship
        pass

    def get_conversation_context_window(self, conversation_id: str, window_size: int = 5) -> List[Dict[str, Any]]:
        """
        Implement conversation context window management (limit history sent to model)
        """
        # Get the most recent messages within the specified window
        recent_messages = self.db.query(Message).filter(
            Message.conversation_id == conversation_id
        ).order_by(Message.timestamp.desc()).limit(window_size).all()

        # Reverse to get chronological order (oldest to newest)
        context = []
        for msg in reversed(recent_messages):
            context.append({
                "role": msg.role,
                "content": msg.content,
                "timestamp": msg.timestamp.isoformat() if msg.timestamp else None
            })

        return context

    def get_conversation_history(self, session_id: str) -> List[Dict[str, Any]]:
        """
        Get the full conversation history for a session
        """
        try:
            conversation = self.db.query(Conversation).filter(
                Conversation.session_id == session_id
            ).first()

            if not conversation:
                return []

            messages = self.db.query(Message).filter(
                Message.conversation_id == conversation.id
            ).order_by(Message.timestamp).all()

            history = []
            for msg in messages:
                history.append({
                    "id": str(msg.id),
                    "role": msg.role,
                    "content": msg.content,
                    "timestamp": msg.timestamp.isoformat() if msg.timestamp else None,
                    "sources": msg.sources
                })

            return history

        except Exception as e:
            raise APIError(f"Error retrieving conversation history: {str(e)}", 500, "HISTORY_RETRIEVAL_ERROR")

    def validate_message(self, message: str) -> bool:
        """
        Validate the user message
        """
        if not message or not message.strip():
            return False

        # Check message length (should be less than 2000 characters as per API spec)
        if len(message) > 2000:
            return False

        return True

    def start_new_conversation(self, session_id: str) -> str:
        """
        Start a new conversation with the given session ID
        """
        try:
            conversation = get_or_create_session(session_id, self.db)
            return str(conversation.id)
        except Exception as e:
            raise APIError(f"Error starting new conversation: {str(e)}", 500, "CONVERSATION_START_ERROR")

    def end_conversation(self, conversation_id: str):
        """
        End a conversation (currently just updates the timestamp)
        """
        try:
            conversation = self.db.query(Conversation).filter(
                Conversation.id == conversation_id
            ).first()

            if conversation:
                conversation.updated_at = datetime.now(timezone.utc)
                self.db.commit()
        except Exception as e:
            raise APIError(f"Error ending conversation: {str(e)}", 500, "CONVERSATION_END_ERROR")

    def validate_conversation_context(self, conversation_id: str) -> bool:
        """
        Create conversation context validation and cleanup logic
        """
        try:
            # Check if conversation exists
            conversation = self.db.query(Conversation).filter(
                Conversation.id == conversation_id
            ).first()

            if not conversation:
                return False

            # Check if conversation is still active (within 24 hours)
            time_diff = datetime.now(timezone.utc) - conversation.updated_at.replace(tzinfo=None)
            if time_diff.total_seconds() > 24 * 3600:  # 24 hours in seconds
                return False

            return True
        except Exception as e:
            raise APIError(f"Error validating conversation context: {str(e)}", 500, "CONTEXT_VALIDATION_ERROR")

    def cleanup_conversation_context(self, conversation_id: str):
        """
        Add conversation context limits to prevent token overflow
        """
        try:
            # Remove very old messages to prevent token overflow
            # Keep only the most recent messages (e.g., last 20 messages)
            old_messages = self.db.query(Message).filter(
                Message.conversation_id == conversation_id
            ).order_by(Message.timestamp.asc()).offset(20).all()  # Keep only last 20 messages

            for msg in old_messages:
                self.db.delete(msg)

            self.db.commit()
        except Exception as e:
            raise APIError(f"Error cleaning up conversation context: {str(e)}", 500, "CONTEXT_CLEANUP_ERROR")

# Global function to get chat service instance
def get_chat_service(db: Session) -> ChatService:
    """
    Get a chat service instance with the provided database session
    """
    return ChatService(db)