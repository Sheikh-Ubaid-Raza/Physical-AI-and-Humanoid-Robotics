"""
Rate limiting middleware for the RAG Chatbot API
"""

import time
from typing import Dict, Optional
from fastapi import Request, HTTPException
from collections import defaultdict, deque
from datetime import datetime, timedelta
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import JSONResponse


class RateLimiter:
    """
    Rate limiter to prevent API abuse (100 requests per minute per session)
    """
    def __init__(self, requests_per_minute: int = 100, window_size: int = 60):
        self.requests_per_minute = requests_per_minute
        self.window_size = window_size  # in seconds
        self.requests: Dict[str, deque] = defaultdict(deque)  # session_id -> list of request timestamps

    def is_allowed(self, session_id: str) -> bool:
        """
        Check if a request from the given session is allowed
        """
        now = time.time()
        window_start = now - self.window_size

        # Clean up old requests outside the window
        while self.requests[session_id] and self.requests[session_id][0] < window_start:
            self.requests[session_id].popleft()

        # Check if we're under the limit
        if len(self.requests[session_id]) < self.requests_per_minute:
            # Add current request
            self.requests[session_id].append(now)
            return True

        return False

    def get_reset_time(self, session_id: str) -> float:
        """
        Get the time when the rate limit will reset for this session
        """
        if not self.requests[session_id]:
            return 0

        # Find the request that will expire first (the oldest within window)
        oldest_request = self.requests[session_id][0] if self.requests[session_id] else time.time()
        return oldest_request + self.window_size


# Global rate limiter instance
rate_limiter = RateLimiter()


def get_rate_limiter() -> RateLimiter:
    """
    Get the global rate limiter instance
    """
    return rate_limiter


# Rate limiting middleware


class RateLimitMiddleware(BaseHTTPMiddleware):
    """
    Middleware to implement rate limiting based on session ID
    """
    def __init__(self, app, requests_per_minute: int = 100):
        super().__init__(app)
        self.rate_limiter = RateLimiter(requests_per_minute, 60)

    async def dispatch(self, request: Request, call_next):
        # Extract session ID from request
        session_id = None

        # Try to get session ID from various sources
        if request.method == "POST" and request.url.path in ["/api/v1/chat", "/api/v1/ingest"]:
            try:
                # For POST requests, we can access the body
                body_bytes = await request.body()
                if body_bytes:
                    import json
                    body = json.loads(body_bytes.decode())
                    session_id = body.get('session_id', 'anonymous')

                    # Restore the request body for downstream handlers
                    from starlette.datastructures import UploadFile
                    import io
                    request._body = body_bytes
            except:
                session_id = "anonymous"
        else:
            # For other requests, try to get from query params or headers
            session_id = request.query_params.get('session_id') or request.headers.get('X-Session-ID') or "anonymous"

        # Apply rate limiting
        if not self.rate_limiter.is_allowed(session_id):
            reset_time = self.rate_limiter.get_reset_time(session_id)
            retry_after = int(reset_time - time.time())

            return JSONResponse(
                status_code=429,
                content={
                    "error": "RATE_LIMIT_EXCEEDED",
                    "message": f"Rate limit exceeded. Please try again in {retry_after} seconds.",
                    "retry_after": retry_after
                }
            )

        response = await call_next(request)
        return response


# Alternative decorator approach for specific endpoints
def rate_limit(requests_per_minute: int = 100):
    """
    Decorator to apply rate limiting to specific endpoints
    """
    limiter = RateLimiter(requests_per_minute)

    def decorator(func):
        def wrapper(*args, **kwargs):
            # This is a simplified approach - in practice, you'd need to extract session_id from the request
            # The middleware approach is preferred for consistent application
            return func(*args, **kwargs)
        return wrapper
    return decorator