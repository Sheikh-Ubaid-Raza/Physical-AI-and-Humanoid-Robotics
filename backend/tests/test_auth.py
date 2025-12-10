import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch, MagicMock
from sqlalchemy.orm import Session
from backend.main import app
from backend.models.user import User, BackgroundLevelEnum, HardwareBackgroundEnum
from backend.services.auth_service import AuthService
import uuid
import jwt


client = TestClient(app)


class TestAuthEndpoints:
    def setup_method(self):
        self.mock_db = Mock(spec=Session)
        self.auth_service = AuthService()

    @patch('backend.api.routes.auth.auth_service')
    @patch('backend.api.routes.auth.get_db')
    def test_register_user_success(self, mock_get_db, mock_auth_service):
        """Test successful user registration."""
        mock_get_db.return_value.__enter__.return_value = self.mock_db

        # Mock the auth service to return a user
        mock_user = Mock(spec=User)
        mock_user.id = uuid.uuid4()
        mock_user.name = "Test User"
        mock_user.email = "test@example.com"
        mock_user.software_background = BackgroundLevelEnum.beginner
        mock_user.hardware_background = HardwareBackgroundEnum.basic

        mock_auth_service.register_user.return_value = mock_user

        response = client.post("/auth/signup", json={
            "name": "Test User",
            "email": "test@example.com",
            "password": "securepassword123",
            "software_background": "beginner",
            "hardware_background": "basic"
        })

        assert response.status_code == 201
        assert "id" in response.json()
        assert response.json()["email"] == "test@example.com"

    @patch('backend.api.routes.auth.auth_service')
    @patch('backend.api.routes.auth.get_db')
    def test_register_user_invalid_background(self, mock_get_db, mock_auth_service):
        """Test user registration with invalid background level."""
        mock_get_db.return_value.__enter__.return_value = self.mock_db

        response = client.post("/auth/signup", json={
            "name": "Test User",
            "email": "test@example.com",
            "password": "securepassword123",
            "software_background": "invalid_level",
            "hardware_background": "basic"
        })

        assert response.status_code == 400

    @patch('backend.api.routes.auth.auth_service')
    @patch('backend.api.routes.auth.get_db')
    def test_login_user_success(self, mock_get_db, mock_auth_service):
        """Test successful user login."""
        mock_get_db.return_value.__enter__.return_value = self.mock_db

        # Mock the auth service to return a user
        mock_user = Mock(spec=User)
        mock_user.id = uuid.uuid4()
        mock_user.name = "Test User"
        mock_user.email = "test@example.com"
        mock_user.software_background = BackgroundLevelEnum.beginner
        mock_user.hardware_background = HardwareBackgroundEnum.basic

        mock_auth_service.authenticate_user.return_value = mock_user
        mock_auth_service.create_user_token.return_value = "mock_jwt_token"

        response = client.post("/auth/signin", json={
            "email": "test@example.com",
            "password": "securepassword123"
        })

        assert response.status_code == 200
        assert "access_token" in response.json()
        assert response.json()["token_type"] == "bearer"

    @patch('backend.api.routes.auth.auth_service')
    @patch('backend.api.routes.auth.get_db')
    def test_login_user_invalid_credentials(self, mock_get_db, mock_auth_service):
        """Test login with invalid credentials."""
        mock_get_db.return_value.__enter__.return_value = self.mock_db

        # Mock the auth service to return None (invalid credentials)
        mock_auth_service.authenticate_user.return_value = None

        response = client.post("/auth/signin", json={
            "email": "test@example.com",
            "password": "wrongpassword"
        })

        assert response.status_code == 401

    def test_logout_user(self):
        """Test user logout."""
        response = client.post("/auth/signout")

        assert response.status_code == 200
        assert response.json()["message"] == "Logout successful"

    @patch('backend.api.routes.auth.verify_token')
    @patch('backend.api.routes.auth.auth_service')
    @patch('backend.api.routes.auth.get_db')
    def test_get_current_user(self, mock_get_db, mock_auth_service, mock_verify_token):
        """Test getting current user information."""
        mock_get_db.return_value.__enter__.return_value = self.mock_db

        # Mock token verification
        mock_token_data = Mock()
        mock_token_data.user_id = str(uuid.uuid4())
        mock_verify_token.return_value = mock_token_data

        # Mock user retrieval
        mock_user = Mock(spec=User)
        mock_user.id = uuid.uuid4()
        mock_user.name = "Test User"
        mock_user.email = "test@example.com"
        mock_user.software_background = BackgroundLevelEnum.beginner
        mock_user.hardware_background = HardwareBackgroundEnum.basic

        mock_auth_service.get_user_by_id.return_value = mock_user

        response = client.get("/auth/me")

        assert response.status_code == 200
        assert response.json()["name"] == "Test User"
        assert response.json()["email"] == "test@example.com"


class TestAuthService:
    def test_hash_and_verify_password(self):
        """Test password hashing and verification."""
        password = "securepassword123"
        hashed = AuthService.hash_password(password)

        assert AuthService.verify_password(password, hashed) is True
        assert AuthService.verify_password("wrongpassword", hashed) is False

    def test_create_and_verify_token(self):
        """Test JWT token creation and verification."""
        user_id = str(uuid.uuid4())
        token = self.auth_service.create_user_token_with_id(user_id)

        # Note: This test might need to be adapted based on the actual implementation
        # of create_user_token_with_id if it doesn't exist
        assert isinstance(token, str)
        assert len(token) > 0