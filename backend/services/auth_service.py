from typing import Optional
from sqlalchemy.orm import Session
from backend.models.user import User
from backend.utils.security import verify_password, get_password_hash, create_access_token
from datetime import timedelta
import uuid


class AuthService:
    def __init__(self):
        pass

    def register_user(
        self,
        db: Session,
        name: str,
        email: str,
        password: str,
        software_background: str,
        hardware_background: str
    ) -> Optional[User]:
        """
        Register a new user with hashed password and background information.
        """
        # Check if user already exists
        existing_user = db.query(User).filter(User.email == email).first()
        if existing_user:
            return None

        # Hash the password
        hashed_password = get_password_hash(password)

        # Create new user
        user = User(
            name=name,
            email=email,
            password_hash=hashed_password,
            software_background=software_background,
            hardware_background=hardware_background
        )

        db.add(user)
        db.commit()
        db.refresh(user)

        return user

    def authenticate_user(self, db: Session, email: str, password: str) -> Optional[User]:
        """
        Authenticate user by email and password.
        """
        user = db.query(User).filter(User.email == email).first()
        if not user or not verify_password(password, user.password_hash):
            return None
        return user

    def create_user_token(self, user: User) -> str:
        """
        Create JWT token for authenticated user.
        """
        data = {
            "sub": str(user.id),
            "email": user.email
        }
        # Create token that expires in 24 hours
        token = create_access_token(data, expires_delta=timedelta(hours=24))
        return token

    def get_user_by_id(self, db: Session, user_id: str) -> Optional[User]:
        """
        Get user by ID.
        """
        try:
            uuid_user_id = uuid.UUID(user_id)
            return db.query(User).filter(User.id == uuid_user_id).first()
        except ValueError:
            # Invalid UUID format
            return None