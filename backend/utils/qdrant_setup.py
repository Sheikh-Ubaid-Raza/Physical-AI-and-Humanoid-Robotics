"""
Qdrant collection setup for Physical AI & Humanoid Robotics textbook
"""

from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, VectorParamsDiff
from qdrant_client.models import PointStruct
import os
from typing import Optional
import logging


class QdrantSetup:
    """
    Setup and configuration for Qdrant collection
    """

    def __init__(self, url: Optional[str] = None, api_key: Optional[str] = None):
        self.url = url or os.getenv("QDRANT_URL")
        self.api_key = api_key or os.getenv("QDRANT_API_KEY")
        self.collection_name = "physical_ai_book"

        if not self.url:
            raise ValueError("QDRANT_URL environment variable is required")

        # Initialize Qdrant client
        if self.api_key:
            self.client = QdrantClient(
                url=self.url,
                api_key=self.api_key,
                prefer_grpc=False  # Using HTTP for compatibility
            )
        else:
            # For local development without API key
            self.client = QdrantClient(url=self.url)

        self.logger = logging.getLogger(__name__)

    def create_collection(self):
        """
        Create the physical_ai_book collection with proper schema
        """
        try:
            # Check if collection already exists
            collections = self.client.get_collections()
            collection_names = [col.name for col in collections.collections]

            if self.collection_name in collection_names:
                self.logger.info(f"Collection '{self.collection_name}' already exists")
                return

            # Create collection with specified parameters
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=768,  # For Google embedding-001
                    distance=Distance.COSINE
                ),
                # Configure payload schema
                optimizers_config=models.OptimizersConfigDiff(
                    memmap_threshold=20000,
                    indexing_threshold=20000,
                )
            )

            # Create payload indexes for faster filtering
            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="module",
                field_schema=models.PayloadSchemaType.KEYWORD
            )

            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="week",
                field_schema=models.PayloadSchemaType.KEYWORD
            )

            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="chapter_title",
                field_schema=models.PayloadSchemaType.TEXT
            )

            self.logger.info(f"Collection '{self.collection_name}' created successfully")

        except Exception as e:
            self.logger.error(f"Error creating collection: {str(e)}")
            raise e

    def delete_collection(self):
        """
        Delete the collection (useful for development/reset)
        """
        try:
            collections = self.client.get_collections()
            collection_names = [col.name for col in collections.collections]

            if self.collection_name in collection_names:
                self.client.delete_collection(self.collection_name)
                self.logger.info(f"Collection '{self.collection_name}' deleted successfully")
            else:
                self.logger.warning(f"Collection '{self.collection_name}' does not exist")

        except Exception as e:
            self.logger.error(f"Error deleting collection: {str(e)}")
            raise e

    def get_collection_info(self):
        """
        Get information about the collection
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                'status': 'success',
                'collection_name': self.collection_name,
                'vector_size': collection_info.config.params.vectors.size,
                'distance': collection_info.config.params.vectors.distance,
                'points_count': collection_info.count,
                'indexed_vectors_count': collection_info.indexed_vectors_count
            }
        except Exception as e:
            self.logger.error(f"Error getting collection info: {str(e)}")
            return {
                'status': 'error',
                'error': str(e)
            }

    def upsert_points(self, points):
        """
        Upsert multiple points to the collection
        """
        try:
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            return {'status': 'success', 'points_upserted': len(points)}
        except Exception as e:
            self.logger.error(f"Error upserting points: {str(e)}")
            raise e

    def search(self, query_vector, top_k=5, filters=None):
        """
        Search for similar content in the collection
        """
        try:
            if filters:
                results = self.client.search(
                    collection_name=self.collection_name,
                    query_vector=query_vector,
                    limit=top_k,
                    query_filter=filters
                )
            else:
                results = self.client.search(
                    collection_name=self.collection_name,
                    query_vector=query_vector,
                    limit=top_k
                )

            return results
        except Exception as e:
            self.logger.error(f"Error searching collection: {str(e)}")
            raise e


# Global instance
qdrant_setup = QdrantSetup()


def get_qdrant_setup() -> QdrantSetup:
    """
    Get the global Qdrant setup instance
    """
    return qdrant_setup


def initialize_qdrant_collection():
    """
    Initialize the Qdrant collection - call this during application startup
    """
    setup = get_qdrant_setup()
    setup.create_collection()
    return setup