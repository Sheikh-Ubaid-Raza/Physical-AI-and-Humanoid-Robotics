"""
Pytest configuration for the RAG Chatbot API tests
"""
import pytest
from unittest.mock import patch
from sqlalchemy import create_engine, text
from sqlalchemy.orm import sessionmaker
from sqlalchemy.types import TypeDecorator, CHAR
from sqlalchemy.dialects.postgresql import UUID
import uuid


class GUID(TypeDecorator):
    """
    Platform-independent GUID type.

    Uses PostgreSQL's UUID type, otherwise uses CHAR(32), storing as stringified hex values.
    """
    impl = CHAR
    cache_ok = True

    def load_dialect_impl(self, dialect):
        if dialect.name == 'postgresql':
            return dialect.type_descriptor(UUID())
        else:
            return dialect.type_descriptor(CHAR(32))

    def process_bind_param(self, value, dialect):
        if value is None:
            return value
        elif dialect.name == 'postgresql':
            return str(value)
        else:
            if not isinstance(value, uuid.UUID):
                return "%.32x" % uuid.UUID(value).int
            else:
                # hexstring
                return "%.32x" % value.int

    def process_result_value(self, value, dialect):
        if value is None:
            return value
        else:
            if not isinstance(value, uuid.UUID):
                value = uuid.UUID(value)
            return value


@pytest.fixture(scope="session")
def db_engine():
    """
    Create a test database engine that handles UUIDs properly for SQLite
    """
    # Patch the GUID type globally for tests
    with patch('sqlalchemy.dialects.postgresql.UUID', new=GUID):
        engine = create_engine("sqlite:///:memory:")

        # Create tables after patching
        from backend.models.conversation import Base
        Base.metadata.create_all(bind=engine)

        yield engine


@pytest.fixture
def db_session(db_engine):
    """
    Create a test database session
    """
    SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=db_engine)
    session = SessionLocal()

    yield session

    session.close()