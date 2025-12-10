import pytest
from unittest.mock import Mock, patch, MagicMock
from backend.services.personalization_service import PersonalizationService
from backend.models.user import User, BackgroundLevelEnum, HardwareBackgroundEnum
from backend.models.personalized_content import PersonalizedContent
from sqlalchemy.orm import Session
import uuid


class TestPersonalizationService:
    def setup_method(self):
        self.service = PersonalizationService()
        self.mock_db = Mock(spec=Session)

    def test_adjust_content_for_beginner(self):
        """Test content adjustment for beginner level."""
        content = "This is content about AI and RAG systems."
        result = self.service._adjust_for_beginner(content)

        assert "Beginner-Friendly Explanation" in result
        assert "Artificial Intelligence (AI)" in result
        assert "Retrieval-Augmented Generation (RAG)" in result

    def test_adjust_content_for_intermediate(self):
        """Test content adjustment for intermediate level."""
        content = "This is content about API usage."
        result = self.service._adjust_for_intermediate(content)

        assert "Intermediate Level Content" in result
        assert "Application Programming Interface (API)" in result

    def test_adjust_content_for_advanced(self):
        """Test content adjustment for advanced level."""
        content = "This is content about algorithm design."
        result = self.service._adjust_for_advanced(content)

        assert "Advanced Level Content" in result
        assert "algorithm" in content.lower()

    def test_adjust_content_for_unknown_level(self):
        """Test content adjustment for unknown level returns original content."""
        content = "Original content"
        result = self.service.adjust_content_for_background(content, "unknown_level")

        assert result == content

    def test_adjust_content_with_caching(self):
        """Test that caching works correctly."""
        content = "Test content for caching"
        user_id = str(uuid.uuid4())
        chapter_id = str(uuid.uuid4())

        # First call
        result1 = self.service.adjust_content_for_background(
            content, "beginner", user_id, chapter_id
        )

        # Second call with same parameters should use cache
        result2 = self.service.adjust_content_for_background(
            content, "beginner", user_id, chapter_id
        )

        assert result1 == result2
        assert "Beginner-Friendly Explanation" in result1

    def test_get_user_background_level(self):
        """Test getting user background level from database."""
        mock_user = Mock(spec=User)
        mock_user.software_background = BackgroundLevelEnum.beginner

        self.mock_db.query().filter().first.return_value = mock_user

        user_id = str(uuid.uuid4())
        result = self.service.get_user_background_level(self.mock_db, user_id)

        assert result == "beginner"

    def test_save_personalized_content(self):
        """Test saving personalized content to database."""
        user_id = str(uuid.uuid4())
        chapter_id = str(uuid.uuid4())
        content = "Personalized content"

        # Mock the query to return None (no existing content)
        self.mock_db.query().filter().first.return_value = None

        result = self.service.save_personalized_content(
            self.mock_db, user_id, chapter_id, content
        )

        # Verify that add and commit were called
        self.mock_db.add.assert_called_once()
        self.mock_db.commit.assert_called_once()

    def test_get_personalized_content(self):
        """Test retrieving personalized content from database."""
        mock_personalized_content = Mock(spec=PersonalizedContent)
        mock_personalized_content.content = "Cached content"

        self.mock_db.query().filter().first.return_value = mock_personalized_content

        user_id = str(uuid.uuid4())
        chapter_id = str(uuid.uuid4())

        result = self.service.get_personalized_content(self.mock_db, user_id, chapter_id)

        assert result == mock_personalized_content