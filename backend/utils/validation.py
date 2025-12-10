"""
Input validation and sanitization utilities for the RAG Chatbot
"""

import re
from typing import Dict, Any, Optional
from pydantic import BaseModel, validator
from fastapi import HTTPException


class ChatRequestValidator(BaseModel):
    """
    Validator for chat requests with proper input validation
    """
    message: str
    session_id: str
    model: Optional[str] = "gpt-4o"
    conversation_context: Optional[list] = []

    @validator('message')
    def validate_message(cls, v):
        """
        Validate message content
        """
        if not v or not v.strip():
            raise ValueError('Message cannot be empty')

        # Check length
        if len(v) > 2000:  # Max 2000 characters as per requirements
            raise ValueError('Message exceeds maximum length of 2000 characters')

        # Sanitize potentially harmful content
        sanitized = cls.sanitize_input(v)
        return sanitized

    @validator('session_id')
    def validate_session_id(cls, v):
        """
        Validate session ID format
        """
        if not v:
            raise ValueError('Session ID cannot be empty')

        # Check for valid session ID format (allow alphanumeric, hyphens, underscores)
        if not re.match(r'^[a-zA-Z0-9_-]+$', v):
            raise ValueError('Invalid session ID format')

        return v

    @classmethod
    def sanitize_input(cls, input_str: str) -> str:
        """
        Sanitize input to prevent injection attacks
        """
        # Remove potentially harmful patterns
        sanitized = input_str

        # Remove script tags
        sanitized = re.sub(r'<script[^>]*>.*?</script>', '', sanitized, flags=re.IGNORECASE | re.DOTALL)

        # Remove iframe tags
        sanitized = re.sub(r'<iframe[^>]*>.*?</iframe>', '', sanitized, flags=re.IGNORECASE | re.DOTALL)

        # Remove javascript: and vbscript: protocols
        sanitized = re.sub(r'javascript:', '', sanitized, flags=re.IGNORECASE)
        sanitized = re.sub(r'vbscript:', '', sanitized, flags=re.IGNORECASE)

        # Remove data: and file: protocols that could be used for injection
        sanitized = re.sub(r'data:', '', sanitized, flags=re.IGNORECASE)
        sanitized = re.sub(r'file:', '', sanitized, flags=re.IGNORECASE)

        # Remove potential SQL injection patterns
        sql_injection_patterns = [
            r"(?i)(union\s+select)",
            r"(?i)(drop\s+table)",
            r"(?i)(delete\s+from)",
            r"(?i)(insert\s+into)",
            r"(?i)(update\s+\w+\s+set)",
            r"(?i)(exec\s*\()",
            r"(?i)(execute\s*\()",
            r"(?i)(sp_\w+)",
            r"(?i)(xp_\w+)"
        ]

        for pattern in sql_injection_patterns:
            sanitized = re.sub(pattern, '', sanitized)

        return sanitized


class IngestRequestValidator(BaseModel):
    """
    Validator for content ingestion requests
    """
    module: str
    week: str
    chapter_title: str
    content: str
    url: str

    @validator('module', 'week', 'chapter_title', 'content', 'url')
    def validate_required_fields(cls, v):
        """
        Validate that required fields are not empty
        """
        if not v or not v.strip():
            raise ValueError('Field cannot be empty')
        return v.strip()

    @validator('url')
    def validate_url(cls, v):
        """
        Validate URL format
        """
        # Basic URL validation - should start with /docs/ or be a valid HTTP URL
        if not (v.startswith('/docs/') or v.startswith('http')):
            raise ValueError('URL must start with /docs/ or be a valid HTTP URL')

        # Additional sanitization
        sanitized = cls.sanitize_input(v)
        return sanitized

    @validator('content')
    def validate_content_length(cls, v):
        """
        Validate content length
        """
        if len(v) > 10000:  # Max 10000 characters
            raise ValueError('Content exceeds maximum length of 10000 characters')

        # Sanitize content
        sanitized = cls.sanitize_input(v)
        return sanitized

    @classmethod
    def sanitize_input(cls, input_str: str) -> str:
        """
        Sanitize input to prevent injection attacks
        """
        # Remove potentially harmful patterns
        sanitized = input_str

        # Remove script tags
        sanitized = re.sub(r'<script[^>]*>.*?</script>', '', sanitized, flags=re.IGNORECASE | re.DOTALL)

        # Remove iframe tags
        sanitized = re.sub(r'<iframe[^>]*>.*?</iframe>', '', sanitized, flags=re.IGNORECASE | re.DOTALL)

        # Remove javascript: and vbscript: protocols
        sanitized = re.sub(r'javascript:', '', sanitized, flags=re.IGNORECASE)
        sanitized = re.sub(r'vbscript:', '', sanitized, flags=re.IGNORECASE)

        # Remove data: and file: protocols that could be used for injection
        sanitized = re.sub(r'data:', '', sanitized, flags=re.IGNORECASE)
        sanitized = re.sub(r'file:', '', sanitized, flags=re.IGNORECASE)

        # Remove potential SQL injection patterns
        sql_injection_patterns = [
            r"(?i)(union\s+select)",
            r"(?i)(drop\s+table)",
            r"(?i)(delete\s+from)",
            r"(?i)(insert\s+into)",
            r"(?i)(update\s+\w+\s+set)",
            r"(?i)(exec\s*\()",
            r"(?i)(execute\s*\()",
            r"(?i)(sp_\w+)",
            r"(?i)(xp_\w+)"
        ]

        for pattern in sql_injection_patterns:
            sanitized = re.sub(pattern, '', sanitized)

        return sanitized


def validate_chat_request(data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Validate chat request data
    """
    try:
        validator = ChatRequestValidator(**data)
        return validator.dict()
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Invalid request data: {str(e)}")


def validate_ingest_request(data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Validate ingestion request data
    """
    try:
        validator = IngestRequestValidator(**data)
        return validator.dict()
    except Exception as e:
        raise HTTPException(status_code=400, detail=f"Invalid ingestion data: {str(e)}")


def sanitize_user_input(input_str: str) -> str:
    """
    General function to sanitize user input
    """
    if not input_str:
        return input_str

    # Use the sanitization from validators
    return ChatRequestValidator.sanitize_input(input_str)


def is_valid_session_id(session_id: str) -> bool:
    """
    Check if a session ID is valid
    """
    if not session_id:
        return False

    # Check for valid session ID format (alphanumeric, hyphens, underscores)
    return bool(re.match(r'^[a-zA-Z0-9_-]+$', session_id))


def is_valid_url(url: str) -> bool:
    """
    Check if a URL is valid for the textbook
    """
    if not url:
        return False

    # Should start with /docs/ or be a valid HTTP URL
    if url.startswith('/docs/'):
        return True

    # Basic HTTP URL validation
    url_pattern = re.compile(
        r'^https?://'  # http:// or https://
        r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # Domain
        r'localhost|'  # localhost
        r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # IP
        r'(?::\d+)?'  # Port
        r'(?:/?|[/?]\S+)$', re.IGNORECASE)

    return bool(re.match(url_pattern, url))