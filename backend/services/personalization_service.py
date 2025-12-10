from typing import Optional
from sqlalchemy.orm import Session
from backend.models.personalized_content import PersonalizedContent
from backend.models.user import User, BackgroundLevelEnum
import uuid
import time
from functools import wraps


class PersonalizationService:
    def __init__(self):
        # In a real implementation, this might connect to a content personalization API
        # or use a more sophisticated personalization algorithm
        self._cache = {}
        self._cache_ttl = 300  # 5 minutes cache TTL

    def _get_cache_key(self, user_id: str, chapter_id: str, background_level: str) -> str:
        """Generate a cache key for personalized content."""
        return f"personalized_content:{user_id}:{chapter_id}:{background_level}"

    def _is_cache_valid(self, cache_entry: dict) -> bool:
        """Check if cache entry is still valid based on TTL."""
        if not cache_entry or 'timestamp' not in cache_entry:
            return False
        return (time.time() - cache_entry['timestamp']) < self._cache_ttl

    def _get_from_cache(self, user_id: str, chapter_id: str, background_level: str) -> Optional[str]:
        """Get personalized content from cache if valid."""
        key = self._get_cache_key(user_id, chapter_id, background_level)
        cache_entry = self._cache.get(key)
        if self._is_cache_valid(cache_entry):
            return cache_entry['content']
        # Remove expired entry
        if cache_entry:
            del self._cache[key]
        return None

    def _set_in_cache(self, user_id: str, chapter_id: str, background_level: str, content: str):
        """Store personalized content in cache."""
        key = self._get_cache_key(user_id, chapter_id, background_level)
        self._cache[key] = {
            'content': content,
            'timestamp': time.time()
        }

    def adjust_content_for_background(
        self,
        content: str,
        background_level: str,
        user_id: Optional[str] = None,
        chapter_id: Optional[str] = None
    ) -> str:
        """
        Adjust content based on user's background level.
        """
        # Use cache if user_id and chapter_id are provided
        if user_id and chapter_id:
            cached_content = self._get_from_cache(user_id, chapter_id, background_level)
            if cached_content:
                return cached_content

        # Map background level to adjustment strategy
        if background_level.lower() == "beginner":
            adjusted_content = self._adjust_for_beginner(content)
        elif background_level.lower() == "intermediate":
            adjusted_content = self._adjust_for_intermediate(content)
        elif background_level.lower() == "advanced":
            adjusted_content = self._adjust_for_advanced(content)
        else:
            # Return original content if background level is not recognized
            adjusted_content = content

        # Cache the result if user_id and chapter_id are provided
        if user_id and chapter_id:
            self._set_in_cache(user_id, chapter_id, background_level, adjusted_content)

        return adjusted_content

    def _adjust_for_beginner(self, content: str) -> str:
        """
        Add explanations and context for beginner level.
        """
        # This is a simplified implementation
        # In a real system, this would use more sophisticated NLP and personalization logic
        adjusted_content = content

        # Add more explanations and context for beginners
        # For example, expanding technical terms, adding more examples
        if "AI" in adjusted_content:
            adjusted_content = adjusted_content.replace("AI", "Artificial Intelligence (AI)")

        if "RAG" in adjusted_content:
            adjusted_content = adjusted_content.replace("RAG", "Retrieval-Augmented Generation (RAG)")

        if "API" in adjusted_content:
            adjusted_content = adjusted_content.replace("API", "Application Programming Interface (API)")

        if "JSON" in adjusted_content:
            adjusted_content = adjusted_content.replace("JSON", "JavaScript Object Notation (JSON)")

        if "SQL" in adjusted_content:
            adjusted_content = adjusted_content.replace("SQL", "Structured Query Language (SQL)")

        # Simplify complex sentences and add more explanations
        # For now, we'll add beginner-friendly explanations at the beginning
        beginner_prefix = (
            "# Beginner-Friendly Explanation\n\n"
            "This content has been adjusted for beginners. "
            "Technical terms are explained, and concepts are broken down into simpler parts.\n\n"
        )

        return f"{beginner_prefix}{adjusted_content}\n\n*This content has been adjusted for beginners with additional explanations and context.*"

    def _adjust_for_intermediate(self, content: str) -> str:
        """
        Provide balanced content for intermediate level.
        """
        # Add moderate technical detail and context
        intermediate_prefix = (
            "# Intermediate Level Content\n\n"
            "This content provides a balanced level of technical detail suitable for intermediate learners.\n\n"
        )

        # Expand some technical terms but not all
        adjusted_content = content
        if "API" in adjusted_content and "Application Programming Interface" not in adjusted_content:
            adjusted_content = adjusted_content.replace("API", "Application Programming Interface (API)")

        return f"{intermediate_prefix}{adjusted_content}\n\n*This content is tailored for intermediate learners.*"

    def _adjust_for_advanced(self, content: str) -> str:
        """
        Provide detailed, technical content for advanced level.
        """
        # Add advanced technical details and deeper explanations
        advanced_prefix = (
            "# Advanced Level Content\n\n"
            "This content includes advanced technical details, implementation considerations, and in-depth analysis.\n\n"
        )

        # Add more technical depth to the content
        adjusted_content = content
        if "algorithm" in adjusted_content.lower():
            adjusted_content += "\n\n## Implementation Considerations\nFor advanced users, consider the computational complexity, memory requirements, and potential optimizations for this algorithm."

        if "system" in adjusted_content.lower():
            adjusted_content += "\n\n## System Architecture\nAdvanced users should consider scalability, fault tolerance, and performance implications of this system design."

        return f"{advanced_prefix}{adjusted_content}\n\n*This content includes advanced technical details and concepts.*"

    def save_personalized_content(
        self,
        db: Session,
        user_id: str,
        chapter_id: str,
        content: str
    ) -> Optional[PersonalizedContent]:
        """
        Save personalized content to database for future retrieval.
        """
        try:
            uuid_user_id = uuid.UUID(user_id)
            uuid_chapter_id = uuid.UUID(chapter_id) if chapter_id and len(chapter_id) == 36 else chapter_id
        except ValueError:
            return None

        # Check if personalized content already exists for this user and chapter
        existing_content = db.query(PersonalizedContent).filter(
            PersonalizedContent.user_id == uuid_user_id,
            PersonalizedContent.chapter_id == uuid_chapter_id
        ).first()

        if existing_content:
            # Update existing content
            existing_content.content = content
            db.commit()
            db.refresh(existing_content)
            return existing_content
        else:
            # Create new personalized content
            personalized_content = PersonalizedContent(
                user_id=uuid_user_id,
                chapter_id=uuid_chapter_id,
                content=content
            )
            db.add(personalized_content)
            db.commit()
            db.refresh(personalized_content)
            return personalized_content

    def get_personalized_content(
        self,
        db: Session,
        user_id: str,
        chapter_id: str
    ) -> Optional[PersonalizedContent]:
        """
        Retrieve personalized content from database.
        """
        try:
            uuid_user_id = uuid.UUID(user_id)
            uuid_chapter_id = uuid.UUID(chapter_id) if chapter_id and len(chapter_id) == 36 else chapter_id
        except ValueError:
            return None

        return db.query(PersonalizedContent).filter(
            PersonalizedContent.user_id == uuid_user_id,
            PersonalizedContent.chapter_id == uuid_chapter_id
        ).first()

    def get_user_background_level(
        self,
        db: Session,
        user_id: str
    ) -> Optional[str]:
        """
        Get user's background level from database.
        """
        try:
            uuid_user_id = uuid.UUID(user_id)
        except ValueError:
            return None

        user = db.query(User).filter(User.id == uuid_user_id).first()
        if user:
            return user.software_background.value
        return None