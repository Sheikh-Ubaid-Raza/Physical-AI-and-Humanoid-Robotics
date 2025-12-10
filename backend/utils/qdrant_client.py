import os
from typing import List, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct, Distance, VectorParams
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class QdrantService:
    def __init__(self, skip_init=False):
        if skip_init:
            # For testing purposes, skip initialization
            self.client = None
            return

        url = os.getenv("QDRANT_URL")
        api_key = os.getenv("QDRANT_API_KEY")

        if not url:
            raise ValueError("QDRANT_URL environment variable is not set")

        # Initialize Qdrant client
        if api_key:
            self.client = QdrantClient(
                url=url,
                api_key=api_key,
                prefer_grpc=False  # Using HTTP for compatibility
            )
        else:
            # For local development without API key
            self.client = QdrantClient(url=url)

        self.collection_name = "physical_ai_book"
        self.vector_size = 768  # For Google embedding-001
        self.distance = Distance.COSINE

        # Initialize the collection if it doesn't exist
        try:
            self._initialize_collection()
        except Exception as e:
            print(f"Warning: Could not initialize Qdrant collection: {e}")
            # In testing environments, we might not have Qdrant running
            # So we'll allow the service to be created without initialization
            pass

    def _initialize_collection(self):
        """
        Initialize the Qdrant collection if it doesn't exist
        """
        if self.client is None:
            # Client is not initialized, skip initialization
            return

        try:
            # Check if collection exists
            self.client.get_collection(collection_name=self.collection_name)
        except:
            # Collection doesn't exist, create it
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.vector_size,
                    distance=self.distance
                ),
                # Define payload schema
                optimizers_config=models.OptimizersConfigDiff(
                    memmap_threshold=20000,
                    indexing_threshold=20000,
                )
            )
            print(f"Created collection: {self.collection_name}")

    def upsert_point(self, point_id: str, vector: List[float], payload: Dict[str, Any]):
        """
        Add or update a point in the collection
        """
        if self.client is None:
            # Client is not initialized, skip operation
            return

        point = PointStruct(
            id=point_id,
            vector=vector,
            payload=payload
        )

        self.client.upsert(
            collection_name=self.collection_name,
            points=[point]
        )

    def search(self, query_vector: List[float], top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in the collection
        """
        if self.client is None:
            # Client is not initialized, return empty results
            return []

        results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=top_k,
            with_payload=True
        )

        # Format results
        formatted_results = []
        for result in results:
            formatted_result = {
                "id": result.id,
                "score": result.score,
                "content": result.payload.get("content", ""),
                "module": result.payload.get("module", ""),
                "week": result.payload.get("week", ""),
                "chapter_title": result.payload.get("chapter_title", ""),
                "url": result.payload.get("url", ""),
                "payload": result.payload
            }
            formatted_results.append(formatted_result)

        return formatted_results

    def batch_upsert(self, points: List[Dict[str, Any]]):
        """
        Add multiple points in a batch operation
        """
        if self.client is None:
            # Client is not initialized, skip operation
            return

        point_structs = []
        for point_data in points:
            point = PointStruct(
                id=point_data["id"],
                vector=point_data["vector"],
                payload=point_data["payload"]
            )
            point_structs.append(point)

        self.client.upsert(
            collection_name=self.collection_name,
            points=point_structs
        )

    def delete_collection(self):
        """
        Delete the entire collection (useful for resets)
        """
        if self.client is None:
            # Client is not initialized, skip operation
            return
        self.client.delete_collection(collection_name=self.collection_name)

# Singleton instance
try:
    qdrant_service = QdrantService()
except Exception:
    # If Qdrant is not available (e.g., in testing), create a service that won't connect
    qdrant_service = QdrantService(skip_init=True)

def get_qdrant_service() -> QdrantService:
    """
    Get the singleton Qdrant service instance
    """
    return qdrant_service