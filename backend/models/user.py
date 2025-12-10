from sqlalchemy import Column, String, DateTime, Enum
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func
from sqlalchemy import CheckConstraint
import uuid
from enum import Enum as PyEnum

Base = declarative_base()

class BackgroundLevelEnum(PyEnum):
    beginner = "beginner"
    intermediate = "intermediate"
    advanced = "advanced"

class HardwareBackgroundEnum(PyEnum):
    none = "none"
    basic = "basic"
    advanced = "advanced"

class User(Base):
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String(255), unique=True, nullable=False)
    name = Column(String(255), nullable=False)
    password_hash = Column(String(255), nullable=False)
    software_background = Column(Enum(BackgroundLevelEnum), nullable=False)
    hardware_background = Column(Enum(HardwareBackgroundEnum), nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), server_default=func.now())

    # Add check constraints for email format and name length
    __table_args__ = (
        CheckConstraint("email ~* '^[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}$'", name='valid_email_format'),
        CheckConstraint("char_length(name) >= 2", name='name_min_length'),
        CheckConstraint("char_length(name) <= 100", name='name_max_length'),
    )