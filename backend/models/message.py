from sqlalchemy import Column, String, DateTime, Text, ForeignKey, JSON
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
import uuid
from backend.models.conversation import Base, GUID

class Message(Base):
    __tablename__ = "messages"

    id = Column(GUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    conversation_id = Column(GUID(as_uuid=True), ForeignKey("conversations.id", ondelete="CASCADE"), nullable=False, index=True)
    role = Column(String(20), nullable=False, index=True)  # 'user' or 'assistant'
    content = Column(Text, nullable=False)
    sources = Column(JSON)  # Optional JSON field for content sources
    timestamp = Column(DateTime(timezone=True), server_default=func.now(), index=True)

    # Relationship
    conversation = relationship("Conversation", back_populates="messages")

    def __repr__(self):
        return f"<Message(id={self.id}, conversation_id={self.conversation_id}, role={self.role}, content={self.content})>"

# Relationship will be defined in conversation.py to avoid circular imports