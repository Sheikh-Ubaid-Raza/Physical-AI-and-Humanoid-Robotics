"""
Enhanced logging utilities for the RAG Chatbot API
"""

import logging
import json
from datetime import datetime
from typing import Dict, Any
import os
from pathlib import Path


class APILogger:
    """
    Enhanced logger for API requests and responses with debugging capabilities
    """
    def __init__(self, name: str = "rag_chatbot_api", log_level: str = "INFO"):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(getattr(logging, log_level.upper()))

        # Prevent duplicate handlers if logger already has handlers
        if not self.logger.handlers:
            # Create logs directory if it doesn't exist
            logs_dir = Path("logs")
            logs_dir.mkdir(exist_ok=True)

            # File handler for detailed logs
            file_handler = logging.FileHandler(logs_dir / "api.log")
            file_formatter = logging.Formatter(
                "%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s"
            )
            file_handler.setFormatter(file_formatter)

            # Console handler for development
            console_handler = logging.StreamHandler()
            console_formatter = logging.Formatter(
                "%(levelname)s - %(message)s"
            )
            console_handler.setFormatter(console_formatter)

            self.logger.addHandler(file_handler)
            self.logger.addHandler(console_handler)

    def log_request(self, method: str, url: str, headers: Dict[str, str], body: str = "", session_id: str = ""):
        """
        Log incoming API requests
        """
        self.logger.info(
            f"REQUEST: {method} {url} | Session: {session_id} | Body: {body[:500]}..." if len(body) > 500 else f"REQUEST: {method} {url} | Session: {session_id} | Body: {body}"
        )

    def log_response(self, status_code: int, response_body: str = "", execution_time: float = 0.0):
        """
        Log API responses
        """
        self.logger.info(
            f"RESPONSE: {status_code} | Time: {execution_time:.3f}s | Body: {response_body[:500]}..." if len(response_body) > 500 else f"RESPONSE: {status_code} | Time: {execution_time:.3f}s | Body: {response_body}"
        )

    def log_error(self, error: Exception, context: str = ""):
        """
        Log errors with context
        """
        self.logger.error(f"ERROR in {context}: {str(error)}", exc_info=True)

    def log_chat_request(self, session_id: str, message: str, model: str = "gpt-4o"):
        """
        Log chat requests with specific details
        """
        self.logger.info(f"CHAT_REQUEST: Session {session_id} | Model: {model} | Message: {message[:200]}...")

    def log_chat_response(self, session_id: str, response: str, sources_count: int = 0, execution_time: float = 0.0):
        """
        Log chat responses with specific details
        """
        self.logger.info(f"CHAT_RESPONSE: Session {session_id} | Sources: {sources_count} | Time: {execution_time:.3f}s | Response: {response[:200]}...")

    def log_ingest_request(self, module: str, week: str, chapter_title: str, content_length: int):
        """
        Log content ingestion requests
        """
        self.logger.info(f"INGEST_REQUEST: Module: {module} | Week: {week} | Chapter: {chapter_title} | Length: {content_length} chars")

    def log_ingest_response(self, status: str, chunk_id: str = "", token_count: int = 0):
        """
        Log content ingestion responses
        """
        self.logger.info(f"INGEST_RESPONSE: Status: {status} | Chunk ID: {chunk_id} | Tokens: {token_count}")

    def log_performance_metric(self, endpoint: str, execution_time: float, response_size: int = 0):
        """
        Log performance metrics for monitoring
        """
        self.logger.info(f"PERFORMANCE: {endpoint} | Time: {execution_time:.3f}s | Size: {response_size} bytes")


# Global logger instance
api_logger = APILogger()


def get_api_logger() -> APILogger:
    """
    Get the global API logger instance
    """
    return api_logger


# Enhanced middleware for logging requests and responses
from fastapi import Request
from fastapi.responses import Response
import time
import json


class LoggingMiddleware:
    """
    Middleware to log all API requests and responses
    """
    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send):
        if scope["type"] != "http":
            return await self.app(scope, receive, send)

        request = Request(scope)
        start_time = time.time()

        # Capture request details
        method = request.method
        url = str(request.url)
        headers = dict(request.headers)

        # Capture request body if needed
        request_body = ""
        if method in ["POST", "PUT", "PATCH"]:
            # Temporarily store body for logging while keeping it available for downstream
            body = b""
            async for chunk in request.stream():
                body += chunk
            request_body = body.decode() if body else ""

            # Restore the request body for downstream handlers
            async def receive_wrapper():
                nonlocal body
                if body:
                    return {"type": "http.request", "body": body, "more_body": False}
                else:
                    return {"type": "http.request", "body": b"", "more_body": False}

            receive = receive_wrapper

        # Store original send to capture response
        response_body = b""
        original_send = send

        async def send_wrapper(message):
            nonlocal response_body
            if message["type"] == "http.response.body":
                response_body += message.get("body", b"")
            await original_send(message)

        # Process request
        await self.app(scope, receive, send_wrapper)

        # Log the request and response
        execution_time = time.time() - start_time
        response_body_str = response_body.decode() if response_body else ""

        # Extract session ID from request or response
        session_id = headers.get('x-session-id', 'unknown')
        if not session_id and request_body:
            try:
                body_json = json.loads(request_body)
                session_id = body_json.get('session_id', 'unknown')
            except:
                session_id = 'unknown'

        # Log using our API logger
        logger = get_api_logger()
        logger.log_request(method, url, headers, request_body, session_id)
        logger.log_response(200, response_body_str, execution_time)  # Note: this assumes success, need to improve

        # Update status code based on response
        # For now, we'll log as 200 - in a real implementation we'd capture the actual status code