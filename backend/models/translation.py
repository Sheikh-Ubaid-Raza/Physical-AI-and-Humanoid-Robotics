import uuid
from sqlalchemy import Column, String, Text, DateTime, Index
from sqlalchemy.dialects.postgresql import UUID as PostgresUUID
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func

Base = declarative_base()

class Translation(Base):
    __tablename__ = "translations"

    id = Column(PostgresUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    chapter_id = Column(String(255), nullable=False, index=True)
    language = Column(String(10), nullable=False, index=True)  # e.g., 'ur' for Urdu
    content = Column(Text, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), server_default=func.now())

# Create composite index for common query pattern: chapter_id + language
Index('idx_translation_chapter_language', 'chapter_id', 'language')