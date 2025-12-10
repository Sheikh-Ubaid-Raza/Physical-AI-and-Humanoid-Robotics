import re
import time
from typing import Optional
from sqlalchemy.orm import Session
from backend.models.translation import Translation
from backend.utils.security import verify_token
import google.generativeai as genai
import os
import uuid


class TranslationService:
    def __init__(self):
        # Initialize Google Gemini API
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise ValueError("GEMINI_API_KEY environment variable is required")

        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel('gemini-2.5-flash')

        # Initialize cache
        self._cache = {}
        self._cache_ttl = 300  # 5 minutes cache TTL

    def _get_cache_key(self, chapter_id: str, content: str, language: str) -> str:
        """Generate a cache key for translated content."""
        import hashlib
        content_hash = hashlib.md5(content.encode()).hexdigest()
        return f"translation:{chapter_id}:{language}:{content_hash}"

    def _is_cache_valid(self, cache_entry: dict) -> bool:
        """Check if cache entry is still valid based on TTL."""
        if not cache_entry or 'timestamp' not in cache_entry:
            return False
        return (time.time() - cache_entry['timestamp']) < self._cache_ttl

    def _get_from_cache(self, chapter_id: str, content: str, language: str) -> Optional[str]:
        """Get translated content from cache if valid."""
        key = self._get_cache_key(chapter_id, content, language)
        cache_entry = self._cache.get(key)
        if self._is_cache_valid(cache_entry):
            return cache_entry['content']
        # Remove expired entry
        if cache_entry:
            del self._cache[key]
        return None

    def _set_in_cache(self, chapter_id: str, content: str, language: str, translated_content: str):
        """Store translated content in cache."""
        key = self._get_cache_key(chapter_id, content, language)
        self._cache[key] = {
            'content': translated_content,
            'timestamp': time.time()
        }

    def translate_to_urdu(
        self,
        content: str,
        preserve_code_blocks: bool = True,
        preserve_technical_terms: bool = True
    ) -> str:
        """
        Translate content to Urdu while preserving code blocks and technical terms.
        """
        # Extract code blocks to preserve them
        code_blocks = []
        if preserve_code_blocks:
            # Find and extract code blocks (fenced code blocks)
            code_pattern = r'(```.*?```|`.*?`|\[code\].*?\[/code\])'
            code_blocks = re.findall(code_pattern, content, re.DOTALL)
            # Replace code blocks with placeholders
            content = re.sub(code_pattern, f'__CODE_BLOCK_{len(code_blocks)-1}__', content)

        # Extract and preserve technical terms if needed
        technical_terms = []
        if preserve_technical_terms:
            # Pattern to identify technical terms (words in ALL CAPS, CamelCase, or with special chars)
            tech_pattern = r'\b[A-Z][A-Z_0-9]*\b|\b[A-Z][a-z]+[A-Z][a-zA-Z]*\b|\b\w+\.\w+\b|\b\w+[-_]\w+\b'
            technical_terms = re.findall(tech_pattern, content)
            # Remove duplicates while preserving order
            technical_terms = list(dict.fromkeys(technical_terms))

            # Replace technical terms with placeholders
            for i, term in enumerate(technical_terms):
                content = content.replace(term, f'__TECH_TERM_{i}__')

        # Prepare the prompt for translation
        prompt = f"""
        Translate the following content to Urdu. Follow these rules:
        1. Translate all text to Urdu
        2. Keep all code blocks, technical terms, and proper English words as they are
        3. Preserve the structure and formatting
        4. Maintain readability in Urdu
        5. Do not translate technical terms like programming concepts, API names, file extensions, etc.
        6. Keep English words that are commonly used in technical contexts in English

        Content to translate:
        {content}
        """

        try:
            # Generate translation using Gemini
            response = self.model.generate_content(prompt)
            translated_content = response.text

            # Restore technical terms if any
            if preserve_technical_terms:
                for i, term in enumerate(reversed(technical_terms)):
                    translated_content = translated_content.replace(
                        f'__TECH_TERM_{len(technical_terms)-1-i}__',
                        term
                    )

            # Restore code blocks if any
            if preserve_code_blocks:
                for i, code_block in enumerate(reversed(code_blocks)):
                    translated_content = translated_content.replace(
                        f'__CODE_BLOCK_{len(code_blocks)-1-i}__',
                        code_block
                    )

            return translated_content
        except Exception as e:
            # If translation fails, return original content with error note
            return f"Translation failed: {str(e)}\n\n{content}"

    def save_translation(
        self,
        db: Session,
        chapter_id: str,
        content: str,
        language: str = "ur"
    ) -> Optional[Translation]:
        """
        Save translation to database for future retrieval.
        """
        # Check if translation already exists for this chapter and language
        existing_translation = db.query(Translation).filter(
            Translation.chapter_id == chapter_id,
            Translation.language == language
        ).first()

        if existing_translation:
            # Update existing translation
            existing_translation.content = content
            db.commit()
            db.refresh(existing_translation)
            return existing_translation
        else:
            # Create new translation
            translation = Translation(
                chapter_id=chapter_id,
                language=language,
                content=content
            )
            db.add(translation)
            db.commit()
            db.refresh(translation)
            return translation

    def get_translation(
        self,
        db: Session,
        chapter_id: str,
        language: str = "ur"
    ) -> Optional[Translation]:
        """
        Retrieve translation from database.
        """
        return db.query(Translation).filter(
            Translation.chapter_id == chapter_id,
            Translation.language == language
        ).first()

    def translate_content_for_user(
        self,
        content: str,
        target_language: str = "ur",
        preserve_code_blocks: bool = True,
        preserve_technical_terms: bool = True,
        chapter_id: Optional[str] = None
    ) -> str:
        """
        Translate content with user-specific options.
        """
        # Use cache if chapter_id is provided
        if chapter_id:
            cached_translation = self._get_from_cache(chapter_id, content, target_language)
            if cached_translation:
                return cached_translation

        if target_language.lower() == "ur":
            result = self.translate_to_urdu(
                content,
                preserve_code_blocks,
                preserve_technical_terms
            )
        else:
            # For now, only Urdu is supported
            # In a real implementation, this would support multiple languages
            result = content

        # Cache the result if chapter_id is provided
        if chapter_id:
            self._set_in_cache(chapter_id, content, target_language, result)

        return result