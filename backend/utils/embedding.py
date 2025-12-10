import os
from typing import List
import google.generativeai as genai
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class EmbeddingService:
    def __init__(self):
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise ValueError("GEMINI_API_KEY environment variable is not set")

        genai.configure(api_key=api_key)
        embedding_model = os.getenv("EMBEDDING_MODEL", "embedding-001")
        self.model_name = f"models/{embedding_model}" if not embedding_model.startswith("models/") else embedding_model
        self.dimensions = 768  # Default for Google embedding-001

    def create_embedding(self, text: str) -> List[float]:
        """
        Create a single embedding for the given text
        """
        try:
            result = genai.embed_content(
                model=self.model_name,
                content=text,
                task_type="RETRIEVAL_DOCUMENT"
            )
            return result['embedding']
        except Exception as e:
            print(f"Error creating embedding: {e}")
            raise e

    def create_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Create embeddings for a list of texts
        """
        embeddings = []
        for text in texts:
            try:
                result = genai.embed_content(
                    model=self.model_name,
                    content=text,
                    task_type="RETRIEVAL_DOCUMENT"
                )
                embeddings.append(result['embedding'])
            except Exception as e:
                print(f"Error creating embedding for text: {e}")
                raise e
        return embeddings

    def generate_embedding_for_content(self, content: str) -> List[float]:
        """
        Generate embedding for content chunks using Google Generative AI
        """
        return self.create_embedding(content)

# Singleton instance
embedding_service = EmbeddingService()

def get_embedding_service() -> EmbeddingService:
    """
    Get the singleton embedding service instance
    """
    return embedding_service