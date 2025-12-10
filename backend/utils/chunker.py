"""
Content chunking utility for RAG pipeline
"""

import re
from typing import List, Tuple
from collections import deque


class ContentChunker:
    """
    Chunk content into 500-1000 token segments with 100-token overlap
    """

    def __init__(self, min_chunk_size: int = 500, max_chunk_size: int = 1000, overlap_size: int = 100):
        self.min_chunk_size = min_chunk_size
        self.max_chunk_size = max_chunk_size
        self.overlap_size = overlap_size

    def estimate_token_count(self, text: str) -> int:
        """
        Estimate token count using rough approximation (1 token ~ 4 characters for English text)
        """
        # This is a rough estimation - for more accurate counting, we could use tiktoken
        # For now, we'll use a simple approach: roughly 1 token per 4 characters
        return len(text) // 4

    def split_by_sentences(self, text: str) -> List[str]:
        """
        Split text into sentences to preserve semantic meaning
        """
        # Split by sentence endings, but preserve the punctuation
        sentences = re.split(r'(?<=[.!?])\s+', text)
        # Clean up empty strings and whitespace
        return [s.strip() for s in sentences if s.strip()]

    def chunk_content(self, content: str, metadata: dict = None) -> List[dict]:
        """
        Chunk content into segments with overlap
        """
        if not content.strip():
            return []

        # First, split into sentences to maintain semantic boundaries
        sentences = self.split_by_sentences(content)
        if not sentences:
            return []

        chunks = []
        current_chunk = ""
        current_token_count = 0

        i = 0
        while i < len(sentences):
            sentence = sentences[i]
            sentence_token_count = self.estimate_token_count(sentence)

            # If adding this sentence would exceed max chunk size
            if current_token_count + sentence_token_count > self.max_chunk_size and current_chunk:
                # Save current chunk
                chunks.append({
                    'content': current_chunk.strip(),
                    'token_count': current_token_count,
                    'metadata': metadata or {},
                    'overlap_with_next': True  # Will be updated later
                })

                # Calculate overlap for next chunk
                # Start the next chunk with overlapping sentences from the current chunk
                overlap_text = self._get_overlap_text(current_chunk, self.overlap_size)
                current_chunk = overlap_text
                current_token_count = self.estimate_token_count(overlap_text)

                # Continue with the same sentence since we need to add it to the new chunk
                continue

            # If we're at the beginning of a new chunk and the sentence is too big
            if not current_chunk and sentence_token_count > self.max_chunk_size:
                # Split the large sentence into smaller chunks
                sub_chunks = self._split_large_sentence(sentence)
                for j, sub_chunk in enumerate(sub_chunks):
                    if j == len(sub_chunks) - 1:  # Last sub-chunk, continue with main loop
                        current_chunk = sub_chunk
                        current_token_count = self.estimate_token_count(sub_chunk)
                    else:  # Add completed sub-chunk
                        chunks.append({
                            'content': sub_chunk,
                            'token_count': self.estimate_token_count(sub_chunk),
                            'metadata': metadata or {},
                            'overlap_with_next': True
                        })
                i += 1
                continue

            # Add sentence to current chunk
            if current_chunk:
                current_chunk += " " + sentence
            else:
                current_chunk = sentence

            current_token_count += sentence_token_count

            # If current chunk is large enough, save it
            if current_token_count >= self.min_chunk_size:
                chunks.append({
                    'content': current_chunk.strip(),
                    'token_count': current_token_count,
                    'metadata': metadata or {},
                    'overlap_with_next': True  # Will be updated later
                })

                # Start next chunk with overlap
                overlap_text = self._get_overlap_text(current_chunk, self.overlap_size)
                current_chunk = overlap_text
                current_token_count = self.estimate_token_count(overlap_text)

            i += 1

        # Add remaining content as final chunk if it's substantial
        if current_chunk and current_token_count > 50:  # At least 50 tokens
            chunks.append({
                'content': current_chunk.strip(),
                'token_count': current_token_count,
                'metadata': metadata or {},
                'overlap_with_next': False
            })

        # Update overlap flags for the last chunk
        if chunks:
            chunks[-1]['overlap_with_next'] = False

        return chunks

    def _get_overlap_text(self, text: str, target_token_count: int) -> str:
        """
        Get the end portion of text that represents approximately target_token_count tokens
        """
        words = text.split()
        if not words:
            return ""

        # Start from the end and accumulate tokens until we reach the target
        current_tokens = 0
        overlap_words = []

        for word in reversed(words):
            word_tokens = self.estimate_token_count(word)
            if current_tokens + word_tokens <= target_token_count:
                overlap_words.insert(0, word)  # Insert at beginning to maintain order
                current_tokens += word_tokens
            else:
                break

        return " ".join(overlap_words)

    def _split_large_sentence(self, sentence: str) -> List[str]:
        """
        Split a sentence that is too large into smaller chunks
        """
        if self.estimate_token_count(sentence) <= self.max_chunk_size:
            return [sentence]

        # Split by commas, semicolons, or other separators
        parts = re.split(r'[,;]\s*', sentence)
        if len(parts) > 1:
            # Try to combine parts back together to reach optimal size
            combined_parts = []
            current_part = ""

            for part in parts:
                test_part = current_part + ", " + part if current_part else part
                if self.estimate_token_count(test_part) <= self.max_chunk_size:
                    current_part = test_part
                else:
                    if current_part:
                        combined_parts.append(current_part)
                    current_part = part

            if current_part:
                combined_parts.append(current_part)

            # Check if all parts are within size limits
            all_within_limit = True
            for part in combined_parts:
                if self.estimate_token_count(part) > self.max_chunk_size:
                    all_within_limit = False
                    break

            if all_within_limit:
                return combined_parts

        # If still too large, split by words
        words = sentence.split()
        chunks = []
        current_chunk = []

        for word in words:
            test_chunk = current_chunk + [word]
            if self.estimate_token_count(" ".join(test_chunk)) <= self.max_chunk_size:
                current_chunk = test_chunk
            else:
                if current_chunk:
                    chunks.append(" ".join(current_chunk))
                current_chunk = [word]

        if current_chunk:
            chunks.append(" ".join(current_chunk))

        return chunks


# Global instance
content_chunker = ContentChunker()


def get_content_chunker() -> ContentChunker:
    """
    Get the global content chunker instance
    """
    return content_chunker