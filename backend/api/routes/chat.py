from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from uuid import UUID, uuid4
import json
from datetime import datetime

# Import models and services
from backend.models.conversation import Conversation
from backend.models.message import Message
from backend.utils.database import get_db, Session
from backend.services.rag_service import get_rag_service, RAGService
from backend.services.session_service import get_or_create_session
from backend.utils.error_handlers import APIError
from backend.utils.logging import get_api_logger

router = APIRouter()

# Request/Response models
class ChatRequest(BaseModel):
    message: str = Field(..., min_length=1, max_length=2000)
    session_id: str
    model: Optional[str] = "gpt-4o"
    conversation_context: Optional[List[Dict[str, str]]] = Field(default_factory=list)

class Source(BaseModel):
    module: str
    week: str
    chapter_title: str
    url: str

class ChatResponse(BaseModel):
    response: str
    sources: List[Source]
    conversation_id: str
    message_id: str


@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    request: ChatRequest,
    db: Session = Depends(get_db)
):
    """
    Process a chat message and return a response using RAG
    """
    try:
        # Get or create session
        conversation = get_or_create_session(request.session_id, db)
        conversation_id = str(conversation.id)

        # Create user message
        user_message = Message(
            conversation_id=conversation.id,
            role="user",
            content=request.message
        )
        db.add(user_message)
        db.commit()
        db.refresh(user_message)

        # Get conversation history for context - prioritize frontend-provided context if available
        formatted_history = []

        # Use conversation context from frontend if provided
        if request.conversation_context:
            formatted_history = request.conversation_context
        else:
            # Fall back to database history (last 5 messages to avoid token overflow)
            conversation_messages = db.query(Message).filter(
                Message.conversation_id == conversation.id
            ).order_by(Message.timestamp).all()

            # Format conversation history for RAG service
            for msg in conversation_messages[:-1]:  # Exclude the current message
                formatted_history.append({
                    "role": msg.role,
                    "content": msg.content
                })

        # Get RAG service and generate response
        rag_service = get_rag_service()
        response_data = await rag_service.generate_response(
            query=request.message,
            conversation_history=formatted_history
        )

        # Create assistant message
        assistant_message = Message(
            conversation_id=conversation.id,
            role="assistant",
            content=response_data["response"],
            sources=response_data["sources"]
        )
        db.add(assistant_message)
        db.commit()
        db.refresh(assistant_message)

        # Format response
        sources = []
        for source in response_data["sources"]:
            sources.append(Source(
                module=source.get("module", ""),
                week=source.get("week", ""),
                chapter_title=source.get("chapter_title", ""),
                url=source.get("url", "")
            ))

        return ChatResponse(
            response=response_data["response"],
            sources=sources,
            conversation_id=conversation_id,
            message_id=str(assistant_message.id)
        )

    except APIError as e:
        raise HTTPException(status_code=e.status_code, detail=e.message)
    except Exception as e:
        raise HTTPException(status_code=500, detail="Internal server error")


# Request/Response models for selected text feature
class SelectedTextRequest(BaseModel):
    message: str = Field(..., min_length=1, max_length=2000)
    session_id: str
    selected_text: str = Field(..., min_length=1, max_length=5000)
    conversation_context: Optional[List[Dict[str, str]]] = Field(default_factory=list)
    model: Optional[str] = "gpt-4o"


class SelectedTextResponse(BaseModel):
    response: str
    sources: List[Source]
    conversation_id: str
    message_id: str


@router.post("/chat/selection", response_model=SelectedTextResponse)
async def selected_text_endpoint(
    request: SelectedTextRequest,
    db: Session = Depends(get_db)
):
    """
    Process selected text as a high-priority query to the AI with priority context.
    """
    logger = get_api_logger()
    logger.log_request("POST", "/chat/selection", {}, f"session_id: {request.session_id}, selected_text_len: {len(request.selected_text)}")

    try:
        # Log the selected text request
        logger.log_chat_request(request.session_id, f"Selected text: {request.selected_text[:100]}...", "gpt-4o")
        # Get or create session
        conversation = get_or_create_session(request.session_id, db)
        conversation_id = str(conversation.id)

        # Create user message with selected text context
        user_message_content = f"Regarding this selected text: '{request.selected_text}', {request.message}"
        user_message = Message(
            conversation_id=conversation.id,
            role="user",
            content=user_message_content
        )
        db.add(user_message)
        db.commit()
        db.refresh(user_message)

        # Get conversation history for context - prioritize frontend-provided context if available
        formatted_history = []

        # Use conversation context from frontend if provided
        if request.conversation_context:
            formatted_history = request.conversation_context
        else:
            # Fall back to database history (last 5 messages to avoid token overflow)
            conversation_messages = db.query(Message).filter(
                Message.conversation_id == conversation.id
            ).order_by(Message.timestamp).all()

            # Format conversation history for RAG service
            for msg in conversation_messages[:-1]:  # Exclude the current message
                formatted_history.append({
                    "role": msg.role,
                    "content": msg.content
                })

        # Get RAG service and generate response
        rag_service = get_rag_service()
        response_data = await rag_service.generate_response(
            query=user_message_content,
            conversation_history=formatted_history
        )

        # Create assistant message
        assistant_message = Message(
            conversation_id=conversation.id,
            role="assistant",
            content=response_data["response"],
            sources=response_data["sources"]
        )
        db.add(assistant_message)
        db.commit()
        db.refresh(assistant_message)

        # Format response
        sources = []
        for source in response_data["sources"]:
            sources.append(Source(
                module=source.get("module", ""),
                week=source.get("week", ""),
                chapter_title=source.get("chapter_title", ""),
                url=source.get("url", "")
            ))

        # Log the response
        logger.log_chat_response(
            request.session_id,
            response_data["response"][:200],
            len(sources)
        )

        return SelectedTextResponse(
            response=response_data["response"],
            sources=sources,
            conversation_id=conversation_id,
            message_id=str(assistant_message.id)
        )

    except APIError as e:
        logger.log_error(e, "selected_text_endpoint APIError")
        raise HTTPException(status_code=e.status_code, detail=e.message)
    except Exception as e:
        logger.log_error(e, "selected_text_endpoint")
        raise HTTPException(status_code=500, detail="Internal server error")


# Import security dependencies for protected endpoints
from backend.utils.security import verify_token, TokenData
from fastapi import Security
from backend.utils.rate_limiter import RateLimiter

# Create rate limiters for personalization and translation endpoints
personalize_limiter = RateLimiter(requests_per_minute=20)  # 20 requests per minute for personalization
translate_limiter = RateLimiter(requests_per_minute=20)    # 20 requests per minute for translation


# Request/Response models for personalization feature
class PersonalizeRequest(BaseModel):
    chapter_id: str
    chapter_content: str
    background_level: Optional[str] = None  # Override user's background level (optional)


class PersonalizeResponse(BaseModel):
    personalized_content: str
    original_background: str
    processing_time: float


@router.post("/personalize", response_model=PersonalizeResponse)
async def personalize_content(
    request: PersonalizeRequest,
    token_data: TokenData = Security(verify_token),  # Requires authentication
    db: Session = Depends(get_db)
):
    """
    Adjust chapter content based on the user's background level.
    """
    # Check rate limit for personalization
    if not personalize_limiter.is_allowed(f"personalize:{token_data.user_id}"):
        raise HTTPException(
            status_code=429,
            detail="Too many personalization requests. Please try again later."
        )

    from backend.services.personalization_service import PersonalizationService
    logger = get_api_logger()
    logger.log_request("POST", "/personalize", {}, f"user_id: {token_data.user_id}, chapter_id: {request.chapter_id}")

    try:
        # Log the personalization request
        logger.log_performance_metric("/personalize", 0, len(request.chapter_content))
        # Get user's background level from database or use the override
        personalization_service = PersonalizationService()

        if request.background_level:
            # Use the provided background level override
            background_level = request.background_level
        else:
            # Get user's background level from database
            user_bg_level = personalization_service.get_user_background_level(db, token_data.user_id)
            if not user_bg_level:
                raise HTTPException(status_code=404, detail="User background level not found")
            background_level = user_bg_level

        # Adjust content based on background level
        import time
        start_time = time.time()

        personalized_content = personalization_service.adjust_content_for_background(
            content=request.chapter_content,
            background_level=background_level,
            user_id=token_data.user_id,
            chapter_id=request.chapter_id
        )

        processing_time = time.time() - start_time

        # Save personalized content to database for future retrieval
        personalization_service.save_personalized_content(
            db=db,
            user_id=token_data.user_id,
            chapter_id=request.chapter_id,
            content=personalized_content
        )

        # Log the response
        logger.log_performance_metric("/personalize", processing_time, len(personalized_content))

        return PersonalizeResponse(
            personalized_content=personalized_content,
            original_background=background_level,
            processing_time=processing_time
        )

    except HTTPException:
        logger.log_error(HTTPException, "personalize_content HTTPException")
        raise
    except Exception as e:
        logger.log_error(e, "personalize_content")
        raise HTTPException(status_code=500, detail="Internal server error")


# Request/Response models for translation feature
class TranslateRequest(BaseModel):
    chapter_id: str
    chapter_content: str
    target_language: str = "ur"  # Default to Urdu


class TranslateResponse(BaseModel):
    translated_content: str
    original_language: str = "en"  # Assuming original is always English
    target_language: str
    processing_time: float
    preserved_elements: dict = {}  # Metadata about preserved elements like code blocks and technical terms


@router.post("/translate", response_model=TranslateResponse)
async def translate_content(
    request: TranslateRequest,
    token_data: TokenData = Security(verify_token),  # Requires authentication
    db: Session = Depends(get_db)
):
    """
    Translate chapter content to the specified language.
    """
    # Check rate limit for translation
    if not translate_limiter.is_allowed(f"translate:{token_data.user_id}"):
        raise HTTPException(
            status_code=429,
            detail="Too many translation requests. Please try again later."
        )

    from backend.services.translation_service import TranslationService
    logger = get_api_logger()
    logger.log_request("POST", "/translate", {}, f"user_id: {token_data.user_id}, chapter_id: {request.chapter_id}, target_lang: {request.target_language}")

    try:
        # Log the translation request
        logger.log_performance_metric("/translate", 0, len(request.chapter_content))
        # Initialize translation service
        translation_service = TranslationService()

        # Translate content
        import time
        start_time = time.time()

        translated_content = translation_service.translate_content_for_user(
            content=request.chapter_content,
            target_language=request.target_language,
            preserve_code_blocks=True,
            preserve_technical_terms=True,
            chapter_id=request.chapter_id
        )

        processing_time = time.time() - start_time

        # Save translation to database for future retrieval
        translation_service.save_translation(
            db=db,
            chapter_id=request.chapter_id,
            content=translated_content,
            language=request.target_language
        )

        # Prepare metadata about preserved elements
        preserved_elements = {
            "code_blocks_preserved": True,
            "technical_terms_preserved": True,
            "language": request.target_language
        }

        # Log the response
        logger.log_performance_metric("/translate", processing_time, len(translated_content))

        return TranslateResponse(
            translated_content=translated_content,
            target_language=request.target_language,
            processing_time=processing_time,
            preserved_elements=preserved_elements
        )

    except HTTPException:
        logger.log_error(HTTPException, "translate_content HTTPException")
        raise
    except Exception as e:
        logger.log_error(e, "translate_content")
        raise HTTPException(status_code=500, detail="Internal server error")