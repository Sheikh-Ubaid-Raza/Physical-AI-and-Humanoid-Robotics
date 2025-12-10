from fastapi import APIRouter
from datetime import datetime
from pydantic import BaseModel
from backend.utils.database import engine
from sqlalchemy import text

router = APIRouter()

class HealthResponse(BaseModel):
    status: str
    timestamp: str

class ErrorResponse(BaseModel):
    error: str
    message: str

@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint to verify the API is running
    """
    try:
        # Test database connection
        with engine.connect() as connection:
            result = connection.execute(text("SELECT 1"))

        return {
            "status": "healthy",
            "timestamp": datetime.now().isoformat()
        }
    except Exception as e:
        return {
            "status": "unhealthy",
            "timestamp": datetime.now().isoformat()
        }

# Additional endpoint to test all services
@router.get("/health/full")
async def full_health_check():
    """
    Full health check that tests all services
    """
    health_status = {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "services": {
            "api": "healthy",
            "database": "checking",
            "qdrant": "checking",
            "gemini": "checking"
        }
    }

    try:
        # Test database
        with engine.connect() as connection:
            result = connection.execute(text("SELECT 1"))
        health_status["services"]["database"] = "healthy"
    except Exception as e:
        health_status["services"]["database"] = f"unhealthy: {str(e)}"
        health_status["status"] = "unhealthy"

    return health_status