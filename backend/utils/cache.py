"""
Caching utilities for the RAG Chatbot API
"""

import hashlib
import json
import time
from typing import Any, Optional, Dict
from datetime import datetime, timedelta
from functools import wraps
from threading import Lock


class SimpleCache:
    """
    Simple in-memory cache with TTL (Time To Live) expiration
    """
    def __init__(self):
        self._cache: Dict[str, Dict[str, Any]] = {}
        self._lock = Lock()

    def get(self, key: str) -> Optional[Any]:
        """
        Get a value from the cache if it exists and hasn't expired
        """
        with self._lock:
            if key in self._cache:
                entry = self._cache[key]
                if entry['expires_at'] > time.time():
                    return entry['value']
                else:
                    # Entry has expired, remove it
                    del self._cache[key]
        return None

    def set(self, key: str, value: Any, ttl: int = 300) -> None:  # Default 5 minutes TTL
        """
        Set a value in the cache with TTL
        """
        with self._lock:
            self._cache[key] = {
                'value': value,
                'expires_at': time.time() + ttl
            }

    def delete(self, key: str) -> bool:
        """
        Delete a key from the cache
        """
        with self._lock:
            if key in self._cache:
                del self._cache[key]
                return True
            return False

    def clear(self) -> None:
        """
        Clear all entries from the cache
        """
        with self._lock:
            self._cache.clear()

    def cleanup_expired(self) -> int:
        """
        Remove all expired entries from the cache and return count of removed entries
        """
        with self._lock:
            current_time = time.time()
            expired_keys = [key for key, entry in self._cache.items()
                           if entry['expires_at'] <= current_time]

            for key in expired_keys:
                del self._cache[key]

            return len(expired_keys)


# Global cache instance
cache = SimpleCache()


def get_cache() -> SimpleCache:
    """
    Get the global cache instance
    """
    return cache


def cache_result(ttl: int = 300):
    """
    Decorator to cache function results
    """
    def decorator(func):
        @wraps(func)
        async def async_wrapper(*args, **kwargs):
            # Create a cache key from function name and arguments
            cache_key = _generate_cache_key(func.__name__, args, kwargs)

            # Try to get from cache first
            cached_result = cache.get(cache_key)
            if cached_result is not None:
                return cached_result

            # Execute the function
            result = await func(*args, **kwargs)

            # Store in cache
            cache.set(cache_key, result, ttl)

            return result

        @wraps(func)
        def sync_wrapper(*args, **kwargs):
            # Create a cache key from function name and arguments
            cache_key = _generate_cache_key(func.__name__, args, kwargs)

            # Try to get from cache first
            cached_result = cache.get(cache_key)
            if cached_result is not None:
                return cached_result

            # Execute the function
            result = func(*args, **kwargs)

            # Store in cache
            cache.set(cache_key, result, ttl)

            return result

        # Return the appropriate wrapper based on whether the function is async
        if func.__code__.co_flags & 0x0080:  # CO_COROUTINE flag
            return async_wrapper
        else:
            return sync_wrapper

    return decorator


def _generate_cache_key(func_name: str, args: tuple, kwargs: dict) -> str:
    """
    Generate a unique cache key from function name and arguments
    """
    # Serialize arguments to create a consistent key
    key_data = {
        'func': func_name,
        'args': args,
        'kwargs': kwargs
    }

    # Create a hash of the serialized data
    serialized = json.dumps(key_data, sort_keys=True, default=str)
    return hashlib.sha256(serialized.encode()).hexdigest()


class ChatResponseCache:
    """
    Specialized cache for chat responses with semantic similarity considerations
    """
    def __init__(self, ttl: int = 600):  # 10 minutes TTL for chat responses
        self.ttl = ttl
        self.cache = SimpleCache()
        self.similarity_threshold = 0.9  # For considering responses similar

    def get_cached_response(self, query: str, session_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a cached response if the query is similar enough to a cached one
        """
        # For now, we'll do exact match caching
        # In a more advanced implementation, we could use semantic similarity
        cache_key = f"chat_response:{session_id}:{hashlib.md5(query.encode()).hexdigest()}"
        return self.cache.get(cache_key)

    def cache_response(self, query: str, session_id: str, response: Dict[str, Any]):
        """
        Cache a chat response
        """
        cache_key = f"chat_response:{session_id}:{hashlib.md5(query.encode()).hexdigest()}"
        self.cache.set(cache_key, response, self.ttl)

    def invalidate_session_cache(self, session_id: str):
        """
        Invalidate all cached responses for a session
        This should be called when a session ends or needs to be refreshed
        """
        # In a real implementation with a more sophisticated cache,
        # we would have a way to bulk delete by prefix
        # For now, this is a placeholder
        pass


class ContentCache:
    """
    Cache for content retrieval operations
    """
    def __init__(self, ttl: int = 900):  # 15 minutes TTL for content
        self.ttl = ttl
        self.cache = SimpleCache()

    def get_cached_content(self, query_embedding: list, top_k: int = 5) -> Optional[list]:
        """
        Get cached content results for an embedding query
        """
        # Create cache key from embedding and parameters
        embedding_hash = hashlib.md5(str(query_embedding).encode()).hexdigest()
        cache_key = f"content_retrieval:{embedding_hash}:{top_k}"
        return self.cache.get(cache_key)

    def cache_content_results(self, query_embedding: list, top_k: int, results: list):
        """
        Cache content retrieval results
        """
        embedding_hash = hashlib.md5(str(query_embedding).encode()).hexdigest()
        cache_key = f"content_retrieval:{embedding_hash}:{top_k}"
        self.cache.set(cache_key, results, self.ttl)

    def invalidate_content_cache(self):
        """
        Invalidate all content cache (e.g., when content is updated)
        """
        # In a real implementation, we would selectively clear content-related cache entries
        # For now, this is a placeholder
        pass


# Global instances
chat_response_cache = ChatResponseCache()
content_cache = ContentCache()


def get_chat_response_cache() -> ChatResponseCache:
    """
    Get the global chat response cache instance
    """
    return chat_response_cache


def get_content_cache() -> ContentCache:
    """
    Get the global content cache instance
    """
    return content_cache


# FastAPI integration
from fastapi import Request, Response
from starlette.middleware.base import BaseHTTPMiddleware
import asyncio


class CacheMiddleware(BaseHTTPMiddleware):
    """
    Middleware to implement response caching for API endpoints
    """
    def __init__(self, app, cache_instance: SimpleCache = None):
        super().__init__(app)
        self.cache = cache_instance or get_cache()

    async def dispatch(self, request: Request, call_next):
        # For GET requests, we could implement caching
        # For now, we'll focus on POST requests with specific logic
        if request.method in ["GET"]:
            # Create cache key from URL and query parameters
            cache_key = f"http_response:{request.url.path}?{sorted(request.query_params.multi_items())}"

            # Try to get from cache
            cached_response = self.cache.get(cache_key)
            if cached_response:
                return Response(content=cached_response['content'],
                              status_code=cached_response['status_code'],
                              headers=cached_response['headers'])

        # For non-cached requests, proceed normally
        response = await call_next(request)

        # For certain responses, we might want to cache them
        # This is just a basic example - in practice, you'd have more sophisticated rules
        if (request.method == "GET" and
            response.status_code == 200 and
            len(response.body) < 10000):  # Don't cache large responses
            cache_key = f"http_response:{request.url.path}?{sorted(request.query_params.multi_items())}"
            self.cache.set(cache_key, {
                'content': response.body,
                'status_code': response.status_code,
                'headers': dict(response.headers)
            }, ttl=300)  # Cache for 5 minutes

        return response