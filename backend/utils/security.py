"""
Security middleware and utilities for the RAG Chatbot API with Advanced Features
"""

import os
import re
import secrets
import hashlib
import bcrypt
from typing import Optional, List
from fastapi import Request, HTTPException, status, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from jose import JWTError, jwt
from passlib.context import CryptContext
from datetime import datetime, timedelta
from pydantic import BaseModel
from datetime import timezone


class SecurityConfig:
    """
    Security configuration constants
    """
    SECRET_KEY = os.getenv("JWT_SECRET_KEY", "your_32_character_secret_key_here_replace_this")
    ALGORITHM = os.getenv("JWT_ALGORITHM", "HS256")
    ACCESS_TOKEN_EXPIRE_HOURS = int(os.getenv("JWT_EXPIRY_HOURS", "24"))
    MAX_CONTENT_LENGTH = 10 * 1024 * 1024  # 10MB in bytes
    ALLOWED_ORIGINS = [
        "https://sheikh-ubaid-raza.github.io",
        "http://localhost:3000",
        "http://localhost:3001",
        "http://127.0.0.1:3000",
        "http://127.0.0.1:3001"
    ]


class TokenData(BaseModel):
    """
    Data model for JWT token
    """
    user_id: str
    email: str
    exp: Optional[int] = None


# Initialize password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """
    Verify a plain password against a hashed password.
    """
    return pwd_context.verify(plain_password, hashed_password)


def get_password_hash(password: str) -> str:
    """
    Generate a hash for a plain password.
    """
    return pwd_context.hash(password)


def create_access_token(data: dict, expires_delta: Optional[timedelta] = None) -> str:
    """
    Create a JWT access token with optional expiration.
    """
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.now(timezone.utc) + expires_delta
    else:
        expire = datetime.now(timezone.utc) + timedelta(hours=SecurityConfig.ACCESS_TOKEN_EXPIRE_HOURS)

    to_encode.update({"exp": int(expire.timestamp())})
    encoded_jwt = jwt.encode(to_encode, SecurityConfig.SECRET_KEY, algorithm=SecurityConfig.ALGORITHM)
    return encoded_jwt


def verify_token(token: str) -> Optional[TokenData]:
    """
    Verify a JWT token and return the token data if valid.
    """
    try:
        payload = jwt.decode(token, SecurityConfig.SECRET_KEY, algorithms=[SecurityConfig.ALGORITHM])
        user_id: str = payload.get("sub")
        email: str = payload.get("email")
        if user_id is None or email is None:
            return None
        token_data = TokenData(user_id=user_id, email=email, exp=payload.get("exp"))

        # Check if token is expired
        if token_data.exp and datetime.fromtimestamp(token_data.exp, tz=timezone.utc) < datetime.now(timezone.utc):
            return None

        return token_data
    except JWTError:
        return None


class SecurityMiddleware:
    """
    Security middleware for API protection
    """
    def __init__(self):
        self.security = HTTPBearer(auto_error=False)
        self.token_blacklist = set()  # In production, use Redis or database

    def sanitize_input(self, input_str: str) -> str:
        """
        Sanitize input to prevent injection attacks
        """
        if not input_str:
            return input_str

        # Remove potentially harmful patterns
        sanitized = input_str

        # Remove script tags and content
        sanitized = re.sub(r'<script[^>]*>.*?</script>', '', sanitized, flags=re.IGNORECASE | re.DOTALL)
        sanitized = re.sub(r'<iframe[^>]*>.*?</iframe>', '', sanitized, flags=re.IGNORECASE | re.DOTALL)

        # Remove javascript: and vbscript: protocols
        sanitized = re.sub(r'javascript:', '', sanitized, flags=re.IGNORECASE)
        sanitized = re.sub(r'vbscript:', '', sanitized, flags=re.IGNORECASE)

        # Remove data: and file: protocols
        sanitized = re.sub(r'data:', '', sanitized, flags=re.IGNORECASE)
        sanitized = re.sub(r'file:', '', sanitized, flags=re.IGNORECASE)

        # Remove potential SQL injection patterns
        sql_patterns = [
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

        for pattern in sql_patterns:
            sanitized = re.sub(pattern, '', sanitized)

        return sanitized

    def validate_content_length(self, content: str, max_length: int = 2000) -> bool:
        """
        Validate content length to prevent oversized payloads
        """
        return len(content) <= max_length

    def validate_url(self, url: str) -> bool:
        """
        Validate URL format to prevent open redirect vulnerabilities
        """
        if not url:
            return True  # Empty URLs are valid for optional fields

        # Check if it's a relative path (safe)
        if url.startswith('/'):
            return True

        # Basic URL pattern validation
        url_pattern = re.compile(
            r'^https?://'  # http:// or https://
            r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # Domain
            r'localhost|'  # localhost
            r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # IP
            r'(?::\d+)?'  # Port
            r'(?:/?|[/?]\S+)$', re.IGNORECASE)

        return bool(re.match(url_pattern, url))

    def is_valid_session_id(self, session_id: str) -> bool:
        """
        Validate session ID format to prevent session fixation
        """
        if not session_id:
            return False

        # Check if session ID matches expected format (alphanumeric + hyphens/underscores)
        session_pattern = re.compile(r'^[a-zA-Z0-9_-]{10,64}$')
        return bool(re.match(session_pattern, session_id))

    def generate_secure_session_id(self) -> str:
        """
        Generate a cryptographically secure session ID
        """
        return secrets.token_urlsafe(32)

    def hash_sensitive_data(self, data: str) -> str:
        """
        Hash sensitive data before storage
        """
        return hashlib.sha256(data.encode()).hexdigest()

    def create_access_token(self, data: dict, expires_delta: timedelta = None) -> str:
        """
        Create JWT access token (updated for user-based authentication)
        """
        to_encode = data.copy()
        if expires_delta:
            expire = datetime.now(timezone.utc) + expires_delta
        else:
            expire = datetime.now(timezone.utc) + timedelta(hours=SecurityConfig.ACCESS_TOKEN_EXPIRE_HOURS)

        to_encode.update({"exp": int(expire.timestamp())})
        encoded_jwt = jwt.encode(to_encode, SecurityConfig.SECRET_KEY, algorithm=SecurityConfig.ALGORITHM)
        return encoded_jwt

    def verify_token(self, token: str) -> Optional[TokenData]:
        """
        Verify JWT token (updated for user-based authentication)
        """
        try:
            payload = jwt.decode(token, SecurityConfig.SECRET_KEY, algorithms=[SecurityConfig.ALGORITHM])
            user_id: str = payload.get("sub")
            email: str = payload.get("email")
            if user_id is None or email is None:
                return None
            token_data = TokenData(user_id=user_id, email=email, exp=payload.get("exp"))

            # Check if token is expired
            if token_data.exp and datetime.fromtimestamp(token_data.exp, tz=timezone.utc) < datetime.now(timezone.utc):
                return None

            return token_data
        except JWTError:
            return None

    def is_token_blacklisted(self, token: str) -> bool:
        """
        Check if token is blacklisted
        """
        token_hash = self.hash_sensitive_data(token)
        return token_hash in self.token_blacklist

    def blacklist_token(self, token: str):
        """
        Add token to blacklist
        """
        token_hash = self.hash_sensitive_data(token)
        self.token_blacklist.add(token_hash)

    async def validate_request(self, request: Request) -> bool:
        """
        Validate incoming request for security compliance
        """
        # Check content length for POST requests
        if request.method in ["POST", "PUT", "PATCH"]:
            try:
                body_bytes = await request.body()
                if len(body_bytes) > SecurityConfig.MAX_CONTENT_LENGTH:
                    raise HTTPException(status_code=413, detail="Request payload too large")
            except Exception:
                # If we can't read the body, continue with other validations
                pass

        # Validate session ID in headers if present
        session_id = request.headers.get("X-Session-ID")
        if session_id and not self.is_valid_session_id(session_id):
            raise HTTPException(status_code=400, detail="Invalid session ID format")

        # Additional validations can be added here

        return True


class RateLimiter:
    """
    Rate limiting functionality to prevent abuse
    """
    def __init__(self, requests_per_minute: int = 100):
        self.requests_per_minute = requests_per_minute
        self.requests_log = {}  # In production, use Redis or database

    def is_allowed(self, identifier: str) -> bool:
        """
        Check if request from identifier is allowed based on rate limit
        """
        import time
        current_time = time.time()
        window_start = current_time - 60  # 60 seconds window

        if identifier not in self.requests_log:
            self.requests_log[identifier] = []

        # Clean up old requests outside the window
        self.requests_log[identifier] = [
            req_time for req_time in self.requests_log[identifier]
            if req_time > window_start
        ]

        # Check if we're under the limit
        if len(self.requests_log[identifier]) < self.requests_per_minute:
            # Add current request
            self.requests_log[identifier].append(current_time)
            return True

        return False

    def get_reset_time(self, identifier: str) -> int:
        """
        Get time when rate limit will reset for this identifier
        """
        import time
        if identifier not in self.requests_log:
            return 0

        # Find the earliest request in the window that will expire
        if self.requests_log[identifier]:
            earliest_request = min(self.requests_log[identifier])
            return int(earliest_request + 60 - time.time())

        return 0


# Global security instances
security_middleware = SecurityMiddleware()
rate_limiter = RateLimiter(requests_per_minute=100)  # 100 requests per minute per session


def get_security_middleware() -> SecurityMiddleware:
    """
    Get the global security middleware instance
    """
    return security_middleware


def get_rate_limiter() -> RateLimiter:
    """
    Get the global rate limiter instance
    """
    return rate_limiter


# FastAPI Security Middleware
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import JSONResponse
import time


class SecurityMiddlewareWrapper(BaseHTTPMiddleware):
    """
    FastAPI middleware wrapper for security functionality
    """
    def __init__(self, app):
        super().__init__(app)
        self.security_service = get_security_middleware()
        self.rate_limiter = get_rate_limiter()

    async def dispatch(self, request: Request, call_next):
        # Get client identifier (IP + session if available)
        client_ip = request.headers.get("x-forwarded-for", request.client.host)
        session_id = request.headers.get("x-session-id") or request.query_params.get("session_id")
        identifier = f"{client_ip}:{session_id or 'anonymous'}"

        # Check rate limit
        if not self.rate_limiter.is_allowed(identifier):
            return JSONResponse(
                status_code=429,
                content={
                    "error": "RATE_LIMIT_EXCEEDED",
                    "message": "Too many requests. Please try again later.",
                    "retry_after": self.rate_limiter.get_reset_time(identifier)
                }
            )

        # Validate request security
        try:
            await self.security_service.validate_request(request)
        except HTTPException as e:
            return JSONResponse(
                status_code=e.status_code,
                content={"error": "SECURITY_VALIDATION_FAILED", "message": e.detail}
            )

        # Process request
        response = await call_next(request)
        return response


# Input sanitization utilities
def sanitize_user_input(input_str: str) -> str:
    """
    Global function to sanitize user input
    """
    return security_middleware.sanitize_input(input_str)


def validate_content_length(content: str, max_length: int = 2000) -> bool:
    """
    Global function to validate content length
    """
    return security_middleware.validate_content_length(content, max_length)


def validate_url_format(url: str) -> bool:
    """
    Global function to validate URL format
    """
    return security_middleware.validate_url(url)


def is_valid_session_identifier(session_id: str) -> bool:
    """
    Global function to validate session ID format
    """
    return security_middleware.is_valid_session_id(session_id)


# Security decorators
def require_valid_session(func):
    """
    Decorator to require valid session for endpoints
    """
    async def wrapper(*args, **kwargs):
        # Extract session from request
        request = None
        for arg in args:
            if isinstance(arg, Request):
                request = arg
                break

        if request:
            session_id = request.headers.get("X-Session-ID") or request.query_params.get("session_id")
            if not session_id or not is_valid_session_identifier(session_id):
                raise HTTPException(status_code=401, detail="Valid session required")

        return await func(*args, **kwargs)
    return wrapper


def rate_limit(max_requests: int = 100, window_minutes: int = 1):
    """
    Decorator to apply rate limiting to specific endpoints
    """
    def decorator(func):
        async def wrapper(*args, **kwargs):
            # Extract request object to get client identifier
            request = None
            for arg in args:
                if isinstance(arg, Request):
                    request = arg
                    break

            if request:
                client_ip = request.headers.get("x-forwarded-for", request.client.host)
                session_id = request.headers.get("x-session-id") or request.query_params.get("session_id")
                identifier = f"{client_ip}:{session_id or 'anonymous'}"

                limiter = RateLimiter(max_requests)
                if not limiter.is_allowed(identifier):
                    raise HTTPException(
                        status_code=429,
                        detail=f"Rate limit exceeded. Maximum {max_requests} requests per {window_minutes} minute(s)."
                    )

            return await func(*args, **kwargs)
        return wrapper
    return decorator