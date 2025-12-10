import pytest
from unittest.mock import Mock, patch, MagicMock
from backend.services.translation_service import TranslationService
from backend.models.translation import Translation
from sqlalchemy.orm import Session
import os
import uuid


class TestTranslationService:
    def setup_method(self):
        # Set up environment variable for API key
        os.environ["GEMINI_API_KEY"] = "test-api-key"
        self.service = TranslationService()
        self.mock_db = Mock(spec=Session)

    @patch('google.generativeai.GenerativeModel')
    def test_translate_to_urdu_basic(self, mock_model):
        """Test basic Urdu translation functionality."""
        mock_response = Mock()
        mock_response.text = "متن کا ترجمہ"
        mock_model.return_value.generate_content.return_value = mock_response

        content = "This is test content"
        result = self.service.translate_to_urdu(content)

        assert "متن کا ترجمہ" in result

    @patch('google.generativeai.GenerativeModel')
    def test_translate_to_urdu_preserves_code_blocks(self, mock_model):
        """Test that code blocks are preserved during translation."""
        mock_response = Mock()
        mock_response.text = "کوڈ بلاک: __CODE_BLOCK_0__"
        mock_model.return_value.generate_content.return_value = mock_response

        content = "This is text with ```code block```"
        result = self.service.translate_to_urdu(content, preserve_code_blocks=True)

        assert "```code block```" in result

    @patch('google.generativeai.GenerativeModel')
    def test_translate_to_urdu_preserves_technical_terms(self, mock_model):
        """Test that technical terms are preserved during translation."""
        mock_response = Mock()
        mock_response.text = "API کا استعمال: __TECH_TERM_0__"
        mock_model.return_value.generate_content.return_value = mock_response

        content = "Using API in the text"
        result = self.service.translate_to_urdu(content, preserve_technical_terms=True)

        assert "API" in result

    def test_translate_content_for_user_with_caching(self):
        """Test that caching works in translate_content_for_user method."""
        # Mock the translation method to return a fixed result
        with patch.object(self.service, 'translate_to_urdu', return_value="translated content"):
            content = "Test content for caching"
            chapter_id = str(uuid.uuid4())

            # First call
            result1 = self.service.translate_content_for_user(
                content, "ur", True, True, chapter_id
            )

            # Second call with same parameters should use cache
            result2 = self.service.translate_content_for_user(
                content, "ur", True, True, chapter_id
            )

            assert result1 == result2
            assert result1 == "translated content"

    def test_save_translation(self):
        """Test saving translation to database."""
        chapter_id = "test-chapter-id"
        content = "Translated content"
        language = "ur"

        # Mock the query to return None (no existing translation)
        self.mock_db.query().filter().first.return_value = None

        result = self.service.save_translation(
            self.mock_db, chapter_id, content, language
        )

        # Verify that add and commit were called
        self.mock_db.add.assert_called_once()
        self.mock_db.commit.assert_called_once()

    def test_get_translation(self):
        """Test retrieving translation from database."""
        mock_translation = Mock(spec=Translation)
        mock_translation.content = "Cached translation"

        self.mock_db.query().filter().first.return_value = mock_translation

        chapter_id = "test-chapter-id"
        language = "ur"

        result = self.service.get_translation(self.mock_db, chapter_id, language)

        assert result == mock_translation