import re
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from pydantic import BaseModel, Field
from pydantic.functional_validators import field_validator
from typing import Optional
from fastapi.security import OAuth2PasswordBearer
from backend.utils.database import get_db
from backend.models.user import User, BackgroundLevelEnum, HardwareBackgroundEnum
from backend.services.auth_service import AuthService
from backend.utils.security import verify_token, TokenData
from backend.utils.rate_limiter import rate_limiter, RateLimiter

# OAuth2 scheme for token authentication
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/auth/signin")

router = APIRouter(prefix="/auth", tags=["Authentication"])

# Pydantic models for request/response
class UserRegisterRequest(BaseModel):
    name: str = Field(..., min_length=2, max_length=100)
    email: str = Field(..., regex=r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$')
    password: str = Field(..., min_length=8, max_length=128)
    software_background: str  # beginner, intermediate, advanced
    hardware_background: str  # none, basic, advanced

    @field_validator('name')
    def validate_name(cls, v):
        if not v.strip():
            raise ValueError('Name cannot be empty or whitespace only')
        # Remove any potentially dangerous characters
        sanitized = re.sub(r'[<>"\']', '', v).strip()
        if len(sanitized) < 2:
            raise ValueError('Name must be at least 2 characters after sanitization')
        return sanitized

    @field_validator('email')
    def validate_email(cls, v):
        if not v.strip():
            raise ValueError('Email cannot be empty')
        # Basic sanitization
        sanitized = v.strip().lower()
        return sanitized

    @field_validator('password')
    def validate_password(cls, v):
        if not v:
            raise ValueError('Password cannot be empty')
        # Check for common weak passwords
        common_weak_passwords = ['password', '12345678', 'qwerty123', 'admin123']
        if v.lower() in common_weak_passwords:
            raise ValueError('Password is too common, please choose a stronger password')
        return v

    @field_validator('software_background', 'hardware_background')
    def validate_background_levels(cls, v):
        if v not in ['beginner', 'intermediate', 'advanced', 'none', 'basic']:
            raise ValueError('Invalid background level provided')
        return v


class UserLoginRequest(BaseModel):
    email: str = Field(..., regex=r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$')
    password: str = Field(..., min_length=1, max_length=128)

    @field_validator('email')
    def validate_email(cls, v):
        if not v.strip():
            raise ValueError('Email cannot be empty')
        # Basic sanitization
        sanitized = v.strip().lower()
        return sanitized


class UserResponse(BaseModel):
    id: str
    name: str
    email: str
    software_background: str
    hardware_background: str

    class Config:
        from_attributes = True


class LoginResponse(BaseModel):
    access_token: str
    token_type: str = "bearer"
    user: UserResponse


class CurrentUserResponse(BaseModel):
    id: str
    name: str
    email: str
    software_background: str
    hardware_background: str

    class Config:
        from_attributes = True


# Initialize auth service
auth_service = AuthService()


# Create specific rate limiters for different endpoints
signup_limiter = RateLimiter(requests_per_minute=10)  # Lower rate for signup to prevent abuse
login_limiter = RateLimiter(requests_per_minute=20)   # Moderate rate for login
general_auth_limiter = RateLimiter(requests_per_minute=100)  # General auth rate limit

@router.post("/signup", response_model=UserResponse, status_code=status.HTTP_201_CREATED)
async def register_user(request: UserRegisterRequest, db: Session = Depends(get_db)):
    """
    Register a new user with background information.
    """
    # Validate background level enums
    try:
        software_bg = BackgroundLevelEnum(request.software_background)
        hardware_bg = HardwareBackgroundEnum(request.hardware_background)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid background level provided"
        )

    # Check rate limit for registration
    if not signup_limiter.is_allowed(f"register:{request.email}"):
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail="Too many registration attempts. Please try again later."
        )

    # Attempt to register the user
    user = auth_service.register_user(
        db=db,
        name=request.name,
        email=request.email,
        password=request.password,
        software_background=request.software_background,
        hardware_background=request.hardware_background
    )

    if not user:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="Email already registered"
        )

    return user


@router.post("/signin", response_model=LoginResponse)
async def login_user(request: UserLoginRequest, db: Session = Depends(get_db)):
    """
    Authenticate user and return JWT token.
    """
    # Check rate limit for login
    if not login_limiter.is_allowed(f"login:{request.email}"):
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail="Too many login attempts. Please try again later."
        )

    # Authenticate the user
    user = auth_service.authenticate_user(
        db=db,
        email=request.email,
        password=request.password
    )

    if not user:
        # Still count failed login attempts for rate limiting purposes
        # but also apply additional security by not revealing if email exists
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password"
        )

    # Create token
    token = auth_service.create_user_token(user)

    # Return token and user info
    return LoginResponse(
        access_token=token,
        token_type="bearer",
        user=UserResponse(
            id=str(user.id),
            name=user.name,
            email=user.email,
            software_background=user.software_background.value,
            hardware_background=user.hardware_background.value
        )
    )


@router.post("/signout", status_code=status.HTTP_200_OK)
async def logout_user():
    """
    Logout user (client-side token removal is primary).
    """
    # In a real implementation, you might add the token to a blacklist
    # For now, we just inform the client that logout was successful
    return {"message": "Logout successful"}


async def get_current_user_from_token(
    token: str = Depends(oauth2_scheme),
    db: Session = Depends(get_db)
) -> TokenData:
    """
    Dependency to get current user from token.
    """
    token_data = verify_token(token)
    if not token_data:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Verify user still exists in database
    user = auth_service.get_user_by_id(db, token_data.user_id)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User no longer exists"
        )

    return token_data


@router.get("/me", response_model=CurrentUserResponse)
async def get_current_user(
    token_data: TokenData = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get information about the currently authenticated user.
    """
    # Get user from database
    user = auth_service.get_user_by_id(db, token_data.user_id)

    if not user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )

    return CurrentUserResponse(
        id=str(user.id),
        name=user.name,
        email=user.email,
        software_background=user.software_background.value,
        hardware_background=user.hardware_background.value
    )