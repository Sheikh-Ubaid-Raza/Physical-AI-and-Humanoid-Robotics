"""
Content ingestion service for the RAG Chatbot
Handles the process of parsing, chunking, and storing textbook content in the vector database
"""

import os
import logging
from typing import Dict, Any, List, Optional
from pathlib import Path
from backend.utils.content_parser import get_content_parser
from backend.utils.chunker import get_content_chunker
from backend.utils.embedding import get_embedding_service
from backend.utils.qdrant_client import get_qdrant_service
from backend.utils.error_handlers import APIError


class IngestionService:
    """
    Service class for handling content ingestion pipeline
    """

    def __init__(self):
        self.parser = get_content_parser()
        self.chunker = get_content_chunker()
        self.embedding_service = get_embedding_service()
        self.qdrant_service = get_qdrant_service()
        self.logger = logging.getLogger(__name__)

    def ingest_single_file(self, file_path: str) -> Dict[str, Any]:
        """
        Ingest a single markdown file into the vector database
        """
        try:
            self.logger.info(f"Starting ingestion of file: {file_path}")

            # Parse the content
            parsed_content = self.parser.parse_markdown_file(file_path)
            content = parsed_content['content']
            metadata = parsed_content['metadata']

            # Chunk the content
            chunks = self.chunker.chunk_content(content, metadata)

            self.logger.info(f"Content chunked into {len(chunks)} chunks")

            # Process each chunk
            results = []
            for i, chunk in enumerate(chunks):
                chunk_content = chunk['content']
                chunk_metadata = {**metadata, **chunk.get('metadata', {})}

                # Generate embedding for the chunk
                embedding = self.embedding_service.create_embedding(chunk_content)

                # Generate a unique ID for this chunk
                chunk_id = f"{Path(file_path).stem}_chunk_{i}_{hash(chunk_content) % 1000000}"

                # Store in Qdrant with metadata
                self.qdrant_service.upsert_point(
                    point_id=chunk_id,
                    vector=embedding,
                    payload={
                        'module': chunk_metadata.get('module', 'N/A'),
                        'week': chunk_metadata.get('week', 'N/A'),
                        'chapter_title': chunk_metadata.get('chapter_title', 'N/A'),
                        'content': chunk_content,
                        'url': chunk_metadata.get('url', 'N/A'),
                        'file_path': chunk_metadata.get('file_path', file_path),
                        'token_count': chunk['token_count'],
                        'source_file': Path(file_path).name
                    }
                )

                results.append({
                    'chunk_id': chunk_id,
                    'token_count': chunk['token_count'],
                    'position': i,
                    'status': 'success'
                })

            self.logger.info(f"Successfully ingested {len(results)} chunks from {file_path}")

            return {
                'status': 'success',
                'file_path': file_path,
                'chunks_processed': len(results),
                'results': results
            }

        except Exception as e:
            self.logger.error(f"Error ingesting file {file_path}: {str(e)}")
            raise APIError(f"Error ingesting file {file_path}: {str(e)}", 500, "INGESTION_ERROR")

    def ingest_directory(self, directory_path: str, recursive: bool = True) -> Dict[str, Any]:
        """
        Ingest all markdown files in a directory
        """
        try:
            self.logger.info(f"Starting ingestion of directory: {directory_path}")

            # Parse all markdown files in the directory
            parsed_contents = self.parser.parse_directory(directory_path, recursive)

            total_chunks = 0
            total_files = len(parsed_contents)
            results = []

            for i, parsed_content in enumerate(parsed_contents):
                file_path = parsed_content['metadata']['file_path']
                self.logger.info(f"Ingesting file {i+1}/{total_files}: {file_path}")

                try:
                    # Chunk the content
                    content = parsed_content['content']
                    metadata = parsed_content['metadata']
                    chunks = self.chunker.chunk_content(content, metadata)

                    # Process each chunk
                    file_results = []
                    for j, chunk in enumerate(chunks):
                        chunk_content = chunk['content']
                        chunk_metadata = {**metadata, **chunk.get('metadata', {})}

                        # Generate embedding for the chunk
                        embedding = self.embedding_service.create_embedding(chunk_content)

                        # Generate a unique ID for this chunk
                        chunk_id = f"{Path(file_path).stem}_chunk_{j}_{hash(chunk_content) % 1000000}"

                        # Store in Qdrant with metadata
                        self.qdrant_service.upsert_point(
                            point_id=chunk_id,
                            vector=embedding,
                            payload={
                                'module': chunk_metadata.get('module', 'N/A'),
                                'week': chunk_metadata.get('week', 'N/A'),
                                'chapter_title': chunk_metadata.get('chapter_title', 'N/A'),
                                'content': chunk_content,
                                'url': chunk_metadata.get('url', 'N/A'),
                                'file_path': chunk_metadata.get('file_path', file_path),
                                'token_count': chunk['token_count'],
                                'source_file': Path(file_path).name
                            }
                        )

                        file_results.append({
                            'chunk_id': chunk_id,
                            'token_count': chunk['token_count'],
                            'position': j,
                            'status': 'success'
                        })

                    results.append({
                        'file_path': file_path,
                        'chunks_processed': len(file_results),
                        'results': file_results
                    })

                    total_chunks += len(file_results)

                except Exception as e:
                    self.logger.error(f"Error processing file {file_path}: {str(e)}")
                    results.append({
                        'file_path': file_path,
                        'chunks_processed': 0,
                        'status': 'error',
                        'error': str(e)
                    })

            self.logger.info(f"Successfully ingested {total_files} files with {total_chunks} total chunks")

            return {
                'status': 'success',
                'files_processed': total_files,
                'total_chunks': total_chunks,
                'results': results
            }

        except Exception as e:
            self.logger.error(f"Error ingesting directory {directory_path}: {str(e)}")
            raise APIError(f"Error ingesting directory {directory_path}: {str(e)}", 500, "INGESTION_ERROR")

    def validate_content(self, content: str) -> Dict[str, Any]:
        """
        Validate content before ingestion
        """
        validation_result = {
            'valid': True,
            'issues': [],
            'warnings': []
        }

        # Check for empty content
        if not content or not content.strip():
            validation_result['valid'] = False
            validation_result['issues'].append('Content is empty')
            return validation_result

        # Check content length
        if len(content) < 10:  # Minimum 10 characters
            validation_result['warnings'].append('Content is very short (< 10 characters)')

        # Check for proper structure
        if '#' not in content and '##' not in content and '###' not in content:
            validation_result['warnings'].append('Content does not contain headers (may affect chunking)')

        return validation_result

    def sanitize_content(self, content: str) -> str:
        """
        Sanitize content before processing
        """
        # Remove excessive whitespace
        content = ' '.join(content.split())

        # Remove any potentially harmful content (basic sanitization)
        # This is a simple implementation - in production, use a proper sanitizer
        content = content.replace('<script', '&lt;script').replace('</script>', '&lt;/script&gt;')

        return content

    def get_ingestion_statistics(self) -> Dict[str, Any]:
        """
        Get statistics about the ingestion process
        """
        # Get collection info from Qdrant
        try:
            collection_info = self.qdrant_service.client.get_collection("physical_ai_book")
            return {
                'status': 'success',
                'total_documents': collection_info.points_count,
                'vectors_count': collection_info.vectors_count,
                'indexed_vectors_count': collection_info.indexed_vectors_count
            }
        except Exception as e:
            self.logger.error(f"Error getting ingestion statistics: {str(e)}")
            return {
                'status': 'error',
                'error': str(e)
            }


# Global instance
ingestion_service = IngestionService()


def get_ingestion_service() -> IngestionService:
    """
    Get the global ingestion service instance
    """
    return ingestion_service