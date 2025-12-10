"""
Timeout handling utilities for the RAG Chatbot API
"""

import asyncio
import signal
import time
from typing import Callable, Any, Optional, Union
from contextlib import contextmanager
from functools import wraps
import logging
from asyncio import TimeoutError
import google.generativeai as genai


class TimeoutError(Exception):
    """
    Custom timeout exception
    """
    pass


class TimeoutHandler:
    """
    Handler for managing API request timeouts
    """
    def __init__(self, default_timeout: int = 30):  # Default 30 seconds
        self.default_timeout = default_timeout
        self.logger = logging.getLogger(__name__)

    @contextmanager
    def timeout(self, seconds: Optional[int] = None):
        """
        Context manager for implementing timeout
        """
        timeout_seconds = seconds or self.default_timeout

        # Start timer
        start_time = time.time()

        try:
            # Yield control to the wrapped code
            yield
        except asyncio.TimeoutError:
            elapsed = time.time() - start_time
            self.logger.warning(f"Operation timed out after {elapsed:.2f}s (timeout: {timeout_seconds}s)")
            raise TimeoutError(f"Operation timed out after {timeout_seconds} seconds")
        except Exception as e:
            elapsed = time.time() - start_time
            if "timeout" in str(e).lower():
                self.logger.warning(f"Operation timed out after {elapsed:.2f}s (timeout: {timeout_seconds}s)")
                raise TimeoutError(f"Operation timed out after {timeout_seconds} seconds")
            raise

    async def timeout_async(self, coro, seconds: Optional[int] = None):
        """
        Async timeout wrapper for coroutines
        """
        timeout_seconds = seconds or self.default_timeout

        try:
            result = await asyncio.wait_for(coro, timeout=timeout_seconds)
            return result
        except asyncio.TimeoutError:
            self.logger.warning(f"Async operation timed out after {timeout_seconds}s")
            raise TimeoutError(f"Operation timed out after {timeout_seconds} seconds")

    def timeout_decorator(self, seconds: Optional[int] = None):
        """
        Decorator for adding timeout to functions
        """
        def decorator(func):
            @wraps(func)
            def wrapper(*args, **kwargs):
                timeout_seconds = seconds or self.default_timeout

                def target():
                    return func(*args, **kwargs)

                import multiprocessing
                from multiprocessing import Queue, Process

                # Use a separate process to enforce timeout (more reliable than threading)
                result_queue = Queue()
                exception_queue = Queue()

                def worker():
                    try:
                        result = target()
                        result_queue.put(('success', result))
                    except Exception as e:
                        result_queue.put(('error', e))

                process = Process(target=worker)
                process.start()
                process.join(timeout=timeout_seconds)

                if process.is_alive():
                    # Timeout occurred
                    process.terminate()
                    process.join()
                    self.logger.warning(f"Function {func.__name__} timed out after {timeout_seconds}s")
                    raise TimeoutError(f"Function {func.__name__} timed out after {timeout_seconds} seconds")

                if not result_queue.empty():
                    status, result = result_queue.get()
                    if status == 'error':
                        raise result
                    return result
                else:
                    # Process ended without putting result in queue
                    raise TimeoutError(f"Function {func.__name__} timed out after {timeout_seconds} seconds")

            return wrapper
        return decorator

    def timeout_async_decorator(self, seconds: Optional[int] = None):
        """
        Async decorator for adding timeout to async functions
        """
        def decorator(func):
            @wraps(func)
            async def async_wrapper(*args, **kwargs):
                timeout_seconds = seconds or self.default_timeout
                try:
                    return await asyncio.wait_for(func(*args, **kwargs), timeout=timeout_seconds)
                except asyncio.TimeoutError:
                    self.logger.warning(f"Async function {func.__name__} timed out after {timeout_seconds}s")
                    raise TimeoutError(f"Function {func.__name__} timed out after {timeout_seconds} seconds")
            return async_wrapper
        return decorator


# Global timeout handler instance
timeout_handler = TimeoutHandler(default_timeout=30)


def get_timeout_handler() -> TimeoutHandler:
    """
    Get the global timeout handler instance
    """
    return timeout_handler


# FastAPI middleware for request timeout
from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware


class RequestTimeoutMiddleware(BaseHTTPMiddleware):
    """
    Middleware to handle request timeouts globally
    """
    def __init__(self, app, default_timeout: int = 30):
        super().__init__(app)
        self.timeout_handler = TimeoutHandler(default_timeout)

    async def dispatch(self, request: Request, call_next):
        # Get timeout from request header or use default
        timeout_header = request.headers.get('X-Timeout')
        timeout = int(timeout_header) if timeout_header and timeout_header.isdigit() else None

        try:
            # Apply timeout to the request processing
            response = await self.timeout_handler.timeout_async(
                call_next(request),
                seconds=timeout
            )
            return response
        except TimeoutError as e:
            return JSONResponse(
                status_code=408,
                content={
                    "error": "REQUEST_TIMEOUT",
                    "message": "The request took too long to process and timed out",
                    "details": {
                        "timeout_seconds": timeout or self.timeout_handler.default_timeout,
                        "request_path": str(request.url.path)
                    }
                }
            )


# Specific timeout utilities for different operations
class APITimeoutManager:
    """
    Manager for different types of API timeouts
    """
    def __init__(self):
        self.chat_timeout = 45  # 45 seconds for chat operations
        self.ingestion_timeout = 120  # 2 minutes for content ingestion
        self.search_timeout = 10  # 10 seconds for search operations
        self.health_timeout = 5  # 5 seconds for health checks

    async def with_chat_timeout(self, coro):
        """
        Execute coroutine with chat-specific timeout
        """
        return await timeout_handler.timeout_async(coro, seconds=self.chat_timeout)

    async def with_ingestion_timeout(self, coro):
        """
        Execute coroutine with ingestion-specific timeout
        """
        return await timeout_handler.timeout_async(coro, seconds=self.ingestion_timeout)

    async def with_search_timeout(self, coro):
        """
        Execute coroutine with search-specific timeout
        """
        return await timeout_handler.timeout_async(coro, seconds=self.search_timeout)

    async def with_health_timeout(self, coro):
        """
        Execute coroutine with health check-specific timeout
        """
        return await timeout_handler.timeout_async(coro, seconds=self.health_timeout)

    @contextmanager
    def chat_timeout_context(self):
        """
        Context manager for chat operations with appropriate timeout
        """
        with timeout_handler.timeout(seconds=self.chat_timeout):
            yield

    @contextmanager
    def ingestion_timeout_context(self):
        """
        Context manager for ingestion operations with appropriate timeout
        """
        with timeout_handler.timeout(seconds=self.ingestion_timeout):
            yield

    @contextmanager
    def search_timeout_context(self):
        """
        Context manager for search operations with appropriate timeout
        """
        with timeout_handler.timeout(seconds=self.search_timeout):
            yield

    @contextmanager
    def health_timeout_context(self):
        """
        Context manager for health check operations with appropriate timeout
        """
        with timeout_handler.timeout(seconds=self.health_timeout):
            yield


# Global timeout manager
api_timeout_manager = APITimeoutManager()


def get_api_timeout_manager() -> APITimeoutManager:
    """
    Get the global API timeout manager instance
    """
    return api_timeout_manager


# Decorators for specific operations
def with_chat_timeout(seconds: Optional[int] = None):
    """
    Decorator for chat operations with timeout
    """
    timeout_sec = seconds or api_timeout_manager.chat_timeout
    return timeout_handler.timeout_decorator(timeout_sec)


def with_async_chat_timeout(seconds: Optional[int] = None):
    """
    Async decorator for chat operations with timeout
    """
    timeout_sec = seconds or api_timeout_manager.chat_timeout
    return timeout_handler.timeout_async_decorator(timeout_sec)


def with_ingestion_timeout(seconds: Optional[int] = None):
    """
    Decorator for ingestion operations with timeout
    """
    timeout_sec = seconds or api_timeout_manager.ingestion_timeout
    return timeout_handler.timeout_decorator(timeout_sec)


def with_async_ingestion_timeout(seconds: Optional[int] = None):
    """
    Async decorator for ingestion operations with timeout
    """
    timeout_sec = seconds or api_timeout_manager.ingestion_timeout
    return timeout_handler.timeout_async_decorator(timeout_sec)


def with_search_timeout(seconds: Optional[int] = None):
    """
    Decorator for search operations with timeout
    """
    timeout_sec = seconds or api_timeout_manager.search_timeout
    return timeout_handler.timeout_decorator(timeout_sec)


def with_async_search_timeout(seconds: Optional[int] = None):
    """
    Async decorator for search operations with timeout
    """
    timeout_sec = seconds or api_timeout_manager.search_timeout
    return timeout_handler.timeout_async_decorator(timeout_sec)


# Timeout-aware service functions
async def timeout_aware_gemini_call(model: str, prompt: str, timeout: int = 30):
    """
    Make Google Gemini API call with timeout protection
    """
    async def make_call():
        # Create the model instance
        model_instance = genai.GenerativeModel(model_name=model)

        # Create the generation config
        generation_config = genai.types.GenerationConfig(
            temperature=0.3,
            max_output_tokens=1000,
            candidate_count=1
        )

        # Generate content with timeout protection
        response = model_instance.generate_content(
            prompt,
            generation_config=generation_config,
            request_options={'timeout': timeout}
        )
        return response.text

    return await timeout_handler.timeout_async(make_call(), seconds=timeout)


async def timeout_aware_embedding_call(input_text: str, timeout: int = 10):
    """
    Make embedding API call with timeout protection
    """
    async def make_call():
        # Using Google's embedding API instead of OpenAI
        result = genai.embed_content(
            model="models/embedding-001",
            content=input_text,
            task_type="RETRIEVAL_DOCUMENT"
        )
        return result['embedding']

    return await timeout_handler.timeout_async(make_call(), seconds=timeout)


async def timeout_aware_qdrant_search(qdrant_client, vector: list, top_k: int = 5, timeout: int = 10):
    """
    Make Qdrant search with timeout protection
    """
    async def make_call():
        return qdrant_client.search(
            collection_name="physical_ai_book",
            query_vector=vector,
            limit=top_k,
            timeout=timeout
        )

    return await timeout_handler.timeout_async(make_call(), seconds=timeout)


# Utility to check if a function is taking too long
def monitor_execution_time(max_duration: float = 30.0):
    """
    Decorator to monitor and log execution time of functions
    """
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            start_time = time.time()
            try:
                result = func(*args, **kwargs)
                execution_time = time.time() - start_time

                if execution_time > max_duration:
                    logging.warning(
                        f"Function {func.__name__} took {execution_time:.2f}s "
                        f"which exceeds the recommended maximum of {max_duration}s"
                    )
                else:
                    logging.debug(
                        f"Function {func.__name__} completed in {execution_time:.2f}s"
                    )

                return result
            except Exception as e:
                execution_time = time.time() - start_time
                logging.error(
                    f"Function {func.__name__} failed after {execution_time:.2f}s: {str(e)}"
                )
                raise
        return wrapper
    return decorator


# Initialize logging for timeout handler
logging.basicConfig(level=logging.INFO)