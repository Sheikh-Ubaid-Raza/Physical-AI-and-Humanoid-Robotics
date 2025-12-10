"""
Content parser for Docusaurus markdown files
"""

import os
from typing import List, Dict, Any
import re
from pathlib import Path
from datetime import datetime


class ContentParser:
    """
    Parse Docusaurus markdown files to extract content for RAG ingestion
    """

    def __init__(self):
        pass

    def parse_markdown_file(self, file_path: str) -> Dict[str, Any]:
        """
        Parse a single markdown file and extract content with metadata
        """
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract metadata from frontmatter if present
        metadata = self._extract_frontmatter(content)

        # Remove frontmatter from content
        content_without_frontmatter = self._remove_frontmatter(content)

        # Extract title from content if not in metadata
        if 'title' not in metadata:
            metadata['title'] = self._extract_title(content_without_frontmatter)

        # Extract module/week/chapter info from file path
        path_parts = Path(file_path).parts
        module_info = self._extract_module_info_from_path(file_path)

        return {
            'content': content_without_frontmatter.strip(),
            'metadata': {
                **metadata,
                **module_info,
                'file_path': file_path,
                'url': self._convert_path_to_url(file_path)
            }
        }

    def _extract_frontmatter(self, content: str) -> Dict[str, Any]:
        """
        Extract frontmatter from markdown content
        """
        frontmatter_match = re.match(r'^---\s*\n(.*?)\n---\s*\n', content, re.DOTALL)
        if frontmatter_match:
            frontmatter = frontmatter_match.group(1)
            metadata = {}
            for line in frontmatter.split('\n'):
                if ':' in line:
                    key, value = line.split(':', 1)
                    key = key.strip()
                    value = value.strip().strip('"\'')
                    metadata[key] = value
            return metadata
        return {}

    def _remove_frontmatter(self, content: str) -> str:
        """
        Remove frontmatter from content
        """
        frontmatter_match = re.match(r'^---\s*\n(.*?)\n---\s*\n', content, re.DOTALL)
        if frontmatter_match:
            return content[frontmatter_match.end():]
        return content

    def _extract_title(self, content: str) -> str:
        """
        Extract title from markdown content (first H1 header)
        """
        h1_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
        if h1_match:
            return h1_match.group(1).strip()
        return "Untitled"

    def _extract_module_info_from_path(self, file_path: str) -> Dict[str, str]:
        """
        Extract module, week, and chapter info from file path
        """
        path = Path(file_path)
        parts = path.parts

        # Look for patterns like week-01, week-02, etc.
        module = "General"
        week = "General"
        chapter_title = path.stem  # filename without extension

        for part in parts:
            if part.startswith('week-') and len(part) >= 6:
                week_num = part[5:]  # Extract number after 'week-'
                week = f"Week {week_num}"
                module = f"Week {week_num}"
                break
            elif 'tutorial' in part.lower():
                module = "Tutorial"
                break
            elif 'intro' in part.lower():
                module = "Introduction"
                break
            elif 'docs' in part.lower():
                module = "Course Material"
                break

        # If we have a more specific title in the content, use that
        return {
            'module': module,
            'week': week,
            'chapter_title': path.stem  # filename without extension as default
        }

    def extract_content_metadata(self, content: str, file_path: str) -> Dict[str, str]:
        """
        Extract metadata from content and file path for RAG ingestion
        """
        # Extract from file path
        path_metadata = self._extract_module_info_from_path(file_path)

        # Extract from content
        title = self._extract_title(content)

        # Extract URL from file path
        url = self._convert_path_to_url(file_path)

        # Combine all metadata
        metadata = {
            'module': path_metadata['module'],
            'week': path_metadata['week'],
            'chapter_title': title,
            'url': url,
            'file_path': file_path
        }

        return metadata

    def _convert_path_to_url(self, file_path: str) -> str:
        """
        Convert file path to relative URL for the textbook
        """
        path = Path(file_path)
        # Convert to relative path from docs directory
        try:
            # Assuming the file is under docs/ directory
            relative_path = path.relative_to(Path(file_path).parent.parent)  # Go up to docs level
            url = f"/docs/{relative_path.with_suffix('').as_posix()}"
            return url
        except ValueError:
            # If path is not relative to docs, create a generic URL
            stem = path.stem
            return f"/docs/{stem}"

    def validate_content(self, content: str) -> Dict[str, Any]:
        """
        Validate content before processing
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

        # Remove any potential injection attempts
        dangerous_patterns = [
            '<iframe', '<object', '<embed', 'javascript:', 'vbscript:', 'data:',
            'onerror=', 'onload=', 'onclick=', 'onmouseover=', 'onfocus='
        ]

        for pattern in dangerous_patterns:
            content = content.replace(pattern, f'[{pattern.replace("<", "").replace(":", "")} removed]')

        return content

    def parse_directory(self, directory_path: str, recursive: bool = True) -> List[Dict[str, Any]]:
        """
        Parse all markdown files in a directory
        """
        content_items = []

        for root, dirs, files in os.walk(directory_path):
            for file in files:
                if file.endswith('.md') or file.endswith('.mdx'):
                    file_path = os.path.join(root, file)
                    try:
                        parsed_content = self.parse_markdown_file(file_path)
                        content_items.append(parsed_content)
                    except Exception as e:
                        print(f"Error parsing file {file_path}: {e}")

            if not recursive:
                break

        return content_items

    def get_content_hash(self, content: str) -> str:
        """
        Generate a hash for content to track changes
        """
        import hashlib
        return hashlib.md5(content.encode('utf-8')).hexdigest()

    def update_and_reingest_content(self, file_path: str, qdrant_service, embedding_service):
        """
        Update and re-ingest content if it has changed
        """
        # Parse the file
        parsed_content = self.parse_markdown_file(file_path)
        content = parsed_content['content']
        metadata = parsed_content['metadata']

        # Generate hash for the content
        content_hash = self.get_content_hash(content)

        # Check if content has changed by comparing with stored hash
        # For now, we'll just update all content - in a real implementation,
        # we'd check against a database of stored hashes
        chunk_id_prefix = f"content_{Path(file_path).stem}"

        # Chunk the content
        from .chunker import get_content_chunker
        chunker = get_content_chunker()
        chunks = chunker.chunk_content(content, metadata)

        updated_chunks = []
        for i, chunk in enumerate(chunks):
            chunk_content = chunk['content']
            chunk_metadata = {**metadata, **chunk.get('metadata', {})}

            # Generate embedding for the chunk
            embedding = embedding_service.create_embedding(chunk_content)

            # Generate a unique ID for this chunk
            chunk_hash = self.get_content_hash(chunk_content)[:12]
            chunk_id = f"{chunk_id_prefix}_chunk_{i}_{chunk_hash}"

            # Store in Qdrant with metadata
            qdrant_service.upsert_point(
                point_id=chunk_id,
                vector=embedding,
                payload={
                    'module': chunk_metadata.get('module', 'N/A'),
                    'week': chunk_metadata.get('week', 'N/A'),
                    'chapter_title': chunk_metadata.get('chapter_title', 'N/A'),
                    'content': chunk_content,
                    'url': chunk_metadata.get('url', 'N/A'),
                    'file_path': chunk_metadata.get('file_path', file_path),
                    'token_count': chunk.get('token_count', len(chunk_content.split())),
                    'source_file': Path(file_path).name,
                    'content_hash': self.get_content_hash(chunk_content)  # Track content changes
                }
            )

            updated_chunks.append({
                'chunk_id': chunk_id,
                'token_count': chunk.get('token_count', len(chunk_content.split())),
                'position': i,
                'status': 'updated'
            })

        return {
            'file_path': file_path,
            'chunks_updated': len(updated_chunks),
            'results': updated_chunks
        }


    def track_ingestion_progress(self, total_files: int, processed_files: int, current_file: str = None) -> Dict[str, Any]:
        """
        Track progress of content ingestion
        """
        progress_percentage = (processed_files / total_files) * 100 if total_files > 0 else 0

        progress_report = {
            'total_files': total_files,
            'processed_files': processed_files,
            'remaining_files': total_files - processed_files,
            'progress_percentage': round(progress_percentage, 2),
            'status': 'completed' if processed_files >= total_files else 'in_progress',
            'current_file': current_file,
            'timestamp': datetime.now().isoformat()
        }

        return progress_report


# Global instance
content_parser = ContentParser()


def get_content_parser() -> ContentParser:
    """
    Get the global content parser instance
    """
    return content_parser