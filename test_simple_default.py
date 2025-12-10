#!/usr/bin/env python3
"""Test basic SQLAlchemy default behavior"""

from sqlalchemy import Column, String, create_engine, Integer
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
import uuid

Base = declarative_base()

class SimpleModel(Base):
    __tablename__ = 'simple_models'

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    name = Column(String)

# Test creation
obj = SimpleModel(name="test")
print(f"SimpleModel ID: {obj.id}")  # This might be None depending on SQLAlchemy version

# Now test with session
engine = create_engine("sqlite:///:memory:")
Base.metadata.create_all(bind=engine)
Session = sessionmaker(bind=engine)
session = Session()

obj2 = SimpleModel(name="test2")
print(f"SimpleModel ID before add: {obj2.id}")

session.add(obj2)
session.flush()  # This should trigger default generation
print(f"SimpleModel ID after flush: {obj2.id}")