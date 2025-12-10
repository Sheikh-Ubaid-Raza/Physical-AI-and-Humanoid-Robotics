"""
Ingestion API routes for the RAG Chatbot
"""

from fastapi import APIRouter, HTTPException, BackgroundTasks, Depends
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
import asyncio
import logging
from pathlib import Path

from backend.utils.database import get_db
from backend.services.ingestion_service import get_ingestion_service
from backend.utils.error_handlers import APIError
from backend.utils.qdrant_setup import initialize_qdrant_collection


router = APIRouter(tags=["ingestion"])

# Request/Response models
class IngestRequest(BaseModel):
    module: str
    week: str
    chapter_title: str
    content: str
    url: str
    file_path: Optional[str] = None


class IngestBatchRequest(BaseModel):
    chunks: List[IngestRequest]


class IngestResponse(BaseModel):
    status: str
    chunk_id: str
    token_count: int
    module: str
    week: str
    chapter_title: str


class IngestBatchResponse(BaseModel):
    status: str
    chunks_processed: int
    success_count: int
    error_count: int
    results: List[Dict[str, Any]]


class IngestDirectoryRequest(BaseModel):
    directory_path: str
    recursive: Optional[bool] = True


@router.post("/ingest", response_model=IngestResponse)
async def ingest_content(
    request: IngestRequest,
    background_tasks: BackgroundTasks
):
    """
    Ingest a single content chunk into the vector database
    """
    try:
        # Get the ingestion service
        ingestion_service = get_ingestion_service()

        # Validate the content
        validation_result = ingestion_service.validate_content(request.content)
        if not validation_result['valid']:
            raise HTTPException(
                status_code=400,
                detail=f"Invalid content: {validation_result['issues']}"
            )

        # Sanitize the content
        sanitized_content = ingestion_service.sanitize_content(request.content)

        # Create temporary file to work with the existing ingestion service interface
        import tempfile
        import os
        with tempfile.NamedTemporaryFile(mode='w', suffix='.md', delete=False) as temp_file:
            temp_file.write(f"---\ntitle: {request.chapter_title}\nmodule: {request.module}\nweek: {request.week}\n---\n\n{sanitized_content}")
            temp_file_path = temp_file.name

        try:
            # For single chunk ingestion, we'll directly use the embedding and qdrant services
            from backend.utils.embedding import get_embedding_service
            from backend.utils.qdrant_client import get_qdrant_service

            embedding_service = get_embedding_service()
            qdrant_service = get_qdrant_service()

            # Generate embedding for the content
            embedding = embedding_service.create_embedding(sanitized_content)

            # Generate a unique ID for this chunk
            import hashlib
            content_hash = hashlib.md5(sanitized_content.encode()).hexdigest()[:12]
            chunk_id = f"ingest_{request.module}_{request.week}_{content_hash}"

            # Store in Qdrant with metadata
            qdrant_service.upsert_point(
                point_id=chunk_id,
                vector=embedding,
                payload={
                    'module': request.module,
                    'week': request.week,
                    'chapter_title': request.chapter_title,
                    'content': sanitized_content,
                    'url': request.url,
                    'file_path': request.file_path or "manual_ingestion",
                    'token_count': len(sanitized_content.split()),  # Rough token count
                    'source_file': "manual_ingestion"
                }
            )

            return IngestResponse(
                status="success",
                chunk_id=chunk_id,
                token_count=len(sanitized_content.split()),
                module=request.module,
                week=request.week,
                chapter_title=request.chapter_title
            )
        finally:
            # Clean up temp file if it exists
            if 'temp_file_path' in locals():
                os.unlink(temp_file_path)

    except HTTPException:
        raise
    except Exception as e:
        logging.error(f"Error in ingest_content: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Ingestion failed: {str(e)}")


@router.post("/ingest-batch", response_model=IngestBatchResponse)
async def ingest_batch_content(
    request: IngestBatchRequest,
    background_tasks: BackgroundTasks
):
    """
    Ingest multiple content chunks in a batch
    """
    try:
        ingestion_service = get_ingestion_service()
        results = []
        success_count = 0
        error_count = 0

        for chunk_request in request.chunks:
            try:
                # Validate the content
                validation_result = ingestion_service.validate_content(chunk_request.content)
                if not validation_result['valid']:
                    results.append({
                        'chunk_id': None,
                        'status': 'error',
                        'error': f"Invalid content: {validation_result['issues']}",
                        'module': chunk_request.module,
                        'week': chunk_request.week
                    })
                    error_count += 1
                    continue

                # Sanitize the content
                sanitized_content = ingestion_service.sanitize_content(chunk_request.content)

                # Generate embedding and store
                from backend.utils.embedding import get_embedding_service
                from backend.utils.qdrant_client import get_qdrant_service

                embedding_service = get_embedding_service()
                qdrant_service = get_qdrant_service()

                # Generate embedding for the content
                embedding = embedding_service.create_embedding(sanitized_content)

                # Generate a unique ID for this chunk
                import hashlib
                content_hash = hashlib.md5(sanitized_content.encode()).hexdigest()[:12]
                chunk_id = f"batch_{chunk_request.module}_{chunk_request.week}_{content_hash}"

                # Store in Qdrant with metadata
                qdrant_service.upsert_point(
                    point_id=chunk_id,
                    vector=embedding,
                    payload={
                        'module': chunk_request.module,
                        'week': chunk_request.week,
                        'chapter_title': chunk_request.chapter_title,
                        'content': sanitized_content,
                        'url': chunk_request.url,
                        'file_path': chunk_request.file_path or "batch_ingestion",
                        'token_count': len(sanitized_content.split()),  # Rough token count
                        'source_file': "batch_ingestion"
                    }
                )

                results.append({
                    'chunk_id': chunk_id,
                    'status': 'success',
                    'token_count': len(sanitized_content.split()),
                    'module': chunk_request.module,
                    'week': chunk_request.week
                })
                success_count += 1

            except Exception as e:
                results.append({
                    'chunk_id': None,
                    'status': 'error',
                    'error': str(e),
                    'module': chunk_request.module,
                    'week': chunk_request.week
                })
                error_count += 1

        return IngestBatchResponse(
            status="success",
            chunks_processed=len(request.chunks),
            success_count=success_count,
            error_count=error_count,
            results=results
        )

    except Exception as e:
        logging.error(f"Error in ingest_batch_content: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Batch ingestion failed: {str(e)}")


@router.post("/ingest-directory")
async def ingest_directory_content(
    request: IngestDirectoryRequest
):
    """
    Ingest all content from a directory
    """
    try:
        ingestion_service = get_ingestion_service()

        # Verify directory exists
        if not Path(request.directory_path).is_dir():
            raise HTTPException(
                status_code=400,
                detail=f"Directory does not exist: {request.directory_path}"
            )

        # Perform directory ingestion
        result = await asyncio.get_event_loop().run_in_executor(
            None,
            lambda: ingestion_service.ingest_directory(request.directory_path, request.recursive)
        )

        return result

    except HTTPException:
        raise
    except Exception as e:
        logging.error(f"Error in ingest_directory_content: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Directory ingestion failed: {str(e)}")


@router.post("/initialize-db")
async def initialize_database():
    """
    Initialize the Qdrant collection for the textbook content
    """
    try:
        setup = initialize_qdrant_collection()
        return {"status": "success", "message": "Qdrant collection initialized successfully"}
    except Exception as e:
        logging.error(f"Error initializing database: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Database initialization failed: {str(e)}")


@router.get("/ingestion-stats")
async def get_ingestion_stats():
    """
    Get statistics about the ingestion process
    """
    try:
        ingestion_service = get_ingestion_service()
        stats = ingestion_service.get_ingestion_statistics()
        return stats
    except Exception as e:
        logging.error(f"Error getting ingestion stats: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Getting stats failed: {str(e)}")


# Health check endpoint
@router.get("/health")
async def health_check():
    """
    Health check for the ingestion service
    """
    try:
        from backend.utils.qdrant_client import get_qdrant_service
        qdrant_service = get_qdrant_service()

        # Try to get collection info as a health check
        stats = qdrant_service.client.get_collection("physical_ai_book")

        return {
            "status": "healthy",
            "service": "ingestion",
            "vector_store": "qdrant_connected",
            "collection_exists": True
        }
    except Exception as e:
        return {
            "status": "unhealthy",
            "service": "ingestion",
            "error": str(e)
        }