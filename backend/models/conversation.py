from sqlalchemy import Column, String, DateTime
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
import uuid
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import event
from sqlalchemy.orm import attributes

# Handle UUID type for different database dialects
from sqlalchemy.types import TypeDecorator, CHAR
import uuid

class GUID(TypeDecorator):
    """
    Platform-independent GUID type.
    Uses PostgreSQL's UUID type in production, CHAR(32) for SQLite in tests.
    """
    impl = CHAR
    cache_ok = True

    def __init__(self, length=32, as_uuid=True):
        self.impl = CHAR(length)
        self.as_uuid = as_uuid
        super(GUID, self).__init__()

    def load_dialect_impl(self, dialect):
        if dialect.name == 'postgresql':
            from sqlalchemy.dialects.postgresql import UUID
            return dialect.type_descriptor(UUID())
        else:
            return dialect.type_descriptor(CHAR(32))

    def process_bind_param(self, value, dialect):
        if value is None:
            return value
        elif dialect.name == 'postgresql':
            return str(value) if self.as_uuid and isinstance(value, uuid.UUID) else value
        else:
            if self.as_uuid and value and not isinstance(value, uuid.UUID):
                value = uuid.UUID(value)
            if isinstance(value, uuid.UUID):
                return "%.32x" % value.int
            else:
                return value

    def process_result_value(self, value, dialect):
        if value is None:
            return value
        else:
            if self.as_uuid and not isinstance(value, uuid.UUID):
                value = uuid.UUID(value)
            return value


# Use GUID as UUID type
UUID = GUID

Base = declarative_base()

class Conversation(Base):
    __tablename__ = "conversations"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(String(255), nullable=False, index=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now(), index=True)
    updated_at = Column(DateTime(timezone=True), server_default=func.now())

    # Relationship with messages
    messages = relationship("Message", back_populates="conversation", cascade="all, delete-orphan")

    def __repr__(self):
        return f"<Conversation(id={self.id}, session_id={self.session_id})>"


from datetime import datetime, timezone

# Event listener to set defaults if not already set (only for creation, not loading from DB)
@event.listens_for(Conversation, 'init', propagate=True)
def set_conversation_defaults(target, args, kwargs):
    if target.id is None:
        target.id = uuid.uuid4()
    current_time = datetime.now(timezone.utc)
    if target.created_at is None:
        target.created_at = current_time
    if target.updated_at is None:
        target.updated_at = current_time  # Set initial value


# Event listener to update updated_at before update
@event.listens_for(Conversation, 'before_update', propagate=True)
def update_timestamps_before_update(mapper, connection, target):
    target.updated_at = datetime.now(timezone.utc)