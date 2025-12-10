"""
Enhanced error handling utilities for the RAG Chatbot API
"""

from typing import Dict, Any
from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException
import logging
import traceback
from enum import Enum


class ErrorCode(Enum):
    """
    Standardized error codes for the API
    """
    # General errors
    INTERNAL_ERROR = "INTERNAL_ERROR"
    VALIDATION_ERROR = "VALIDATION_ERROR"
    AUTHENTICATION_ERROR = "AUTHENTICATION_ERROR"
    AUTHORIZATION_ERROR = "AUTHORIZATION_ERROR"

    # Chat-specific errors
    CHAT_PROCESSING_ERROR = "CHAT_PROCESSING_ERROR"
    CONVERSATION_NOT_FOUND = "CONVERSATION_NOT_FOUND"
    SESSION_EXPIRED = "SESSION_EXPIRED"

    # Content-related errors
    CONTENT_NOT_FOUND = "CONTENT_NOT_FOUND"
    CONTENT_RETRIEVAL_ERROR = "CONTENT_RETRIEVAL_ERROR"
    EMBEDDING_GENERATION_ERROR = "EMBEDDING_GENERATION_ERROR"

    # Database errors
    DATABASE_CONNECTION_ERROR = "DATABASE_CONNECTION_ERROR"
    DATABASE_QUERY_ERROR = "DATABASE_QUERY_ERROR"

    # External service errors
    EXTERNAL_SERVICE_UNAVAILABLE = "EXTERNAL_SERVICE_UNAVAILABLE"
    RATE_LIMIT_EXCEEDED = "RATE_LIMIT_EXCEEDED"


class APIError(Exception):
    """
    Custom API exception with standardized structure
    """
    def __init__(self, message: str, error_code: ErrorCode, status_code: int = 500, details: Dict[str, Any] = None):
        self.message = message
        self.error_code = error_code.value
        self.status_code = status_code
        self.details = details or {}
        super().__init__(self.message)

    def to_dict(self) -> Dict[str, Any]:
        """
        Convert error to dictionary for JSON response
        """
        return {
            "error": self.error_code,
            "message": self.message,
            "status_code": self.status_code,
            "details": self.details
        }


def handle_validation_error(exc: RequestValidationError, request: Request):
    """
    Handle request validation errors with user-friendly messages
    """
    errors = []
    for error in exc.errors():
        field = " -> ".join(str(loc) for loc in error['loc'])
        message = error['msg']
        error_type = error['type']
        errors.append({
            "field": field,
            "message": message,
            "type": error_type
        })

    return JSONResponse(
        status_code=422,
        content={
            "error": ErrorCode.VALIDATION_ERROR.value,
            "message": "Invalid request parameters provided",
            "details": {
                "errors": errors,
                "request_path": str(request.url.path)
            }
        }
    )


def handle_http_exception(exc: StarletteHTTPException, request: Request):
    """
    Handle HTTP exceptions with user-friendly messages
    """
    error_details = {
        "status_code": exc.status_code,
        "request_path": str(request.url.path),
        "detail": exc.detail
    }

    # Map common HTTP status codes to more user-friendly messages
    if exc.status_code == 404:
        message = "The requested resource was not found. Please check the URL and try again."
    elif exc.status_code == 401:
        message = "Authentication required. Please check your credentials."
    elif exc.status_code == 403:
        message = "Access denied. You don't have permission to access this resource."
    elif exc.status_code == 429:
        message = "Too many requests. Please wait before trying again."
    else:
        message = exc.detail if exc.detail else "An error occurred while processing your request."

    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": ErrorCode.INTERNAL_ERROR.value if exc.status_code >= 500 else ErrorCode.VALIDATION_ERROR.value,
            "message": message,
            "details": error_details
        }
    )


def handle_internal_error(exc: Exception, request: Request):
    """
    Handle internal server errors with user-friendly messages
    """
    # Log the full error for debugging
    logging.error(f"Internal error occurred: {str(exc)}", exc_info=True)

    # For security, don't expose internal error details to users
    return JSONResponse(
        status_code=500,
        content={
            "error": ErrorCode.INTERNAL_ERROR.value,
            "message": "An internal server error occurred. Our team has been notified and is working to resolve the issue.",
            "details": {
                "request_path": str(request.url.path),
                "timestamp": __import__('datetime').datetime.now(__import__('datetime').timezone.utc).isoformat()
            }
        }
    )


def handle_api_error(exc: APIError, request: Request):
    """
    Handle custom API errors
    """
    return JSONResponse(
        status_code=exc.status_code,
        content=exc.to_dict()
    )


def handle_chat_processing_error(exc: Exception, request: Request):
    """
    Handle chat-specific processing errors with user-friendly messages
    """
    logging.error(f"Chat processing error: {str(exc)}", exc_info=True)

    return JSONResponse(
        status_code=500,
        content={
            "error": ErrorCode.CHAT_PROCESSING_ERROR.value,
            "message": "I encountered an issue while processing your message. Please try again or rephrase your question.",
            "details": {
                "request_path": str(request.url.path),
                "timestamp": __import__('datetime').datetime.now(__import__('datetime').timezone.utc).isoformat()
            }
        }
    )


def handle_content_retrieval_error(exc: Exception, request: Request):
    """
    Handle content retrieval errors with user-friendly messages
    """
    logging.error(f"Content retrieval error: {str(exc)}", exc_info=True)

    return JSONResponse(
        status_code=500,
        content={
            "error": ErrorCode.CONTENT_RETRIEVAL_ERROR.value,
            "message": "I'm having trouble accessing the textbook content right now. Please try again in a moment.",
            "details": {
                "request_path": str(request.url.path),
                "timestamp": __import__('datetime').datetime.now(__import__('datetime').timezone.utc).isoformat()
            }
        }
    )


def handle_external_service_error(service_name: str, exc: Exception, request: Request):
    """
    Handle external service (Google AI, Qdrant, etc.) errors with user-friendly messages
    """
    logging.error(f"External service ({service_name}) error: {str(exc)}", exc_info=True)

    service_messages = {
        "Google AI": "The AI service is temporarily unavailable. Please try again in a moment.",
        "Qdrant": "The content database is temporarily unavailable. Please try again.",
        "Neon": "The conversation history service is temporarily unavailable. Your chat history might not be saved."
    }

    message = service_messages.get(service_name, f"The {service_name} service is temporarily unavailable. Please try again.")

    return JSONResponse(
        status_code=503,
        content={
            "error": ErrorCode.EXTERNAL_SERVICE_UNAVAILABLE.value,
            "message": message,
            "details": {
                "service": service_name,
                "request_path": str(request.url.path),
                "timestamp": __import__('datetime').datetime.now(__import__('datetime').timezone.utc).isoformat()
            }
        }
    )


# Error handling middleware for FastAPI
def register_error_handlers(app):
    """
    Register error handlers with the FastAPI app
    """
    @app.exception_handler(RequestValidationError)
    async def validation_exception_handler(request: Request, exc: RequestValidationError):
        return handle_validation_error(exc, request)

    @app.exception_handler(StarletteHTTPException)
    async def http_exception_handler(request: Request, exc: StarletteHTTPException):
        return handle_http_exception(exc, request)

    @app.exception_handler(APIError)
    async def api_exception_handler(request: Request, exc: APIError):
        return handle_api_error(exc, request)

    @app.exception_handler(Exception)
    async def general_exception_handler(request: Request, exc: Exception):
        return handle_internal_error(exc, request)


# Convenience functions for raising common errors
def raise_validation_error(field: str, message: str, details: Dict[str, Any] = None):
    """
    Raise a validation error with user-friendly message
    """
    raise APIError(
        message=f"Validation error in field '{field}': {message}",
        error_code=ErrorCode.VALIDATION_ERROR,
        status_code=422,
        details=details or {"field": field, "issue": message}
    )


def raise_resource_not_found(resource_type: str, identifier: str = None):
    """
    Raise a resource not found error
    """
    message = f"The requested {resource_type} was not found"
    if identifier:
        message += f": {identifier}"

    raise APIError(
        message=message,
        error_code=ErrorCode.CONVERSATION_NOT_FOUND,
        status_code=404,
        details={"resource_type": resource_type, "identifier": identifier}
    )


def raise_rate_limit_error(retry_after: int = 60):
    """
    Raise a rate limit exceeded error
    """
    raise APIError(
        message=f"Rate limit exceeded. Please try again in {retry_after} seconds.",
        error_code=ErrorCode.RATE_LIMIT_EXCEEDED,
        status_code=429,
        details={"retry_after": retry_after}
    )


def raise_session_expired(session_id: str):
    """
    Raise a session expired error
    """
    raise APIError(
        message="Your session has expired. Please start a new conversation.",
        error_code=ErrorCode.SESSION_EXPIRED,
        status_code=401,
        details={"session_id": session_id}
    )


def raise_content_not_found(query: str):
    """
    Raise a content not found error
    """
    raise APIError(
        message=f"No relevant content found for your query: '{query}'. Try rephrasing your question.",
        error_code=ErrorCode.CONTENT_NOT_FOUND,
        status_code=404,
        details={"query": query}
    )


def raise_external_service_unavailable(service_name: str):
    """
    Raise an external service unavailable error
    """
    raise APIError(
        message=f"The {service_name} service is temporarily unavailable. Please try again.",
        error_code=ErrorCode.EXTERNAL_SERVICE_UNAVAILABLE,
        status_code=503,
        details={"service": service_name}
    )


# Custom error response utilities
def create_error_response(error_code: ErrorCode, message: str, status_code: int, details: Dict[str, Any] = None) -> JSONResponse:
    """
    Create a standardized error response
    """
    return JSONResponse(
        status_code=status_code,
        content={
            "error": error_code.value,
            "message": message,
            "details": details or {}
        }
    )


def create_user_friendly_error(error_code: ErrorCode, user_message: str, status_code: int, technical_details: Dict[str, Any] = None) -> JSONResponse:
    """
    Create an error response with user-friendly message while preserving technical details internally
    """
    return JSONResponse(
        status_code=status_code,
        content={
            "error": error_code.value,
            "message": user_message,
            "details": technical_details or {}
        }
    )


# Logging configuration for errors
def setup_error_logging():
    """
    Set up error logging configuration
    """
    # Configure logging to capture errors
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('logs/error.log'),
            logging.StreamHandler()
        ]
    )


# Initialize error logging
setup_error_logging()