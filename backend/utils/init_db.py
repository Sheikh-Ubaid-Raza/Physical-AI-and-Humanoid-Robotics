from backend.utils.database import engine, Base
from backend.models.conversation import Conversation
from backend.models.message import Message
from backend.models.user import User
from backend.models.personalized_content import PersonalizedContent
from backend.models.translation import Translation
from sqlalchemy import text

def init_database():
    """
    Initialize the database by creating all tables
    """
    print("Initializing database...")

    # Create all tables defined in the models
    Base.metadata.create_all(bind=engine)

    print("Database tables created successfully!")

    # Test the connection
    try:
        with engine.connect() as connection:
            result = connection.execute(text("SELECT 1"))
            print("Database connection test successful!")
    except Exception as e:
        print(f"Database connection test failed: {e}")
        raise e

if __name__ == "__main__":
    init_database()