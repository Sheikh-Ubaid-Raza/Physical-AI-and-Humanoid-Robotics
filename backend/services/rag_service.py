from typing import List, Dict, Any, Optional
from backend.utils.embedding import EmbeddingService
from backend.utils.qdrant_client import QdrantService
from backend.utils.timeout_handler import timeout_aware_gemini_call, timeout_aware_embedding_call
import google.generativeai as genai
import os
from dotenv import load_dotenv
import json
import re

# Load environment variables
load_dotenv()

import time

class RAGService:
    def __init__(self):
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise ValueError("GEMINI_API_KEY environment variable is not set")

        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel(os.getenv("MODEL_NAME", "gemini-2.5-flash"))
        self.embedding_service = EmbeddingService()
        self.qdrant_service = QdrantService()
        self.confidence_threshold = 0.6  # As specified in requirements

        # Add caching for RAG responses to optimize performance
        self._response_cache = {}
        self._cache_ttl = 300  # 5 minutes cache TTL for RAG responses

        # Resource monitoring
        self._request_count = 0
        self._total_processing_time = 0.0
        self._cache_hits = 0
        self._cache_misses = 0

    def is_off_topic_query(self, query: str) -> bool:
        """
        Implement off-topic question detection
        """
        # Define off-topic keywords and phrases that indicate questions outside the textbook scope
        off_topic_keywords = [
            'weather', 'joke', 'movie', 'game', 'sport', 'politics', 'celebrity', 'news',
            'stock', 'price', 'buy', 'sell', 'investment', 'crypto', 'bitcoin',
            'personal opinion', 'your favorite', 'what do you think', 'tell me about yourself',
            'current events', 'today\'s date', 'who are you', 'what are you',
            'gossip', 'rumor', 'unverified', 'secret', 'confidential'
        ]

        # Convert query to lowercase for comparison
        query_lower = query.lower()

        # Check if query contains off-topic keywords
        for keyword in off_topic_keywords:
            if keyword in query_lower:
                return True

        # Check if query is too general or not related to AI/robotics/physics
        general_questions = [
            'hello', 'hi', 'how are you', 'what\'s up', 'tell me something',
            'random fact', 'anything interesting', 'help me', 'what can you do'
        ]

        for general_question in general_questions:
            if general_question in query_lower:
                # Check if it's asking about the textbook content specifically
                if not any(topic in query_lower for topic in ['physical ai', 'humanoid', 'robot', 'ai', 'textbook', 'course', 'material', 'content']):
                    return True

        return False

    def _get_cache_key(self, query: str, conversation_history: Optional[List[Dict[str, Any]]]) -> str:
        """
        Generate a cache key for RAG responses.
        """
        import hashlib
        # Create a hash of the query and conversation history
        cache_input = query
        if conversation_history:
            cache_input += str(conversation_history)
        return hashlib.md5(cache_input.encode()).hexdigest()

    def _is_cache_valid(self, cache_entry: dict) -> bool:
        """
        Check if cache entry is still valid based on TTL.
        """
        if not cache_entry or 'timestamp' not in cache_entry:
            return False
        return (time.time() - cache_entry['timestamp']) < self._cache_ttl

    def _get_from_cache(self, query: str, conversation_history: Optional[List[Dict[str, Any]]]) -> Optional[Dict[str, Any]]:
        """
        Get RAG response from cache if valid.
        """
        key = self._get_cache_key(query, conversation_history)
        cache_entry = self._response_cache.get(key)
        if self._is_cache_valid(cache_entry):
            return cache_entry['response']
        # Remove expired entry
        if cache_entry:
            del self._response_cache[key]
        return None

    def _set_in_cache(self, query: str, conversation_history: Optional[List[Dict[str, Any]]], response: Dict[str, Any]):
        """
        Store RAG response in cache.
        """
        key = self._get_cache_key(query, conversation_history)
        self._response_cache[key] = {
            'response': response,
            'timestamp': time.time()
        }

    def retrieve_relevant_content(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve relevant content from the vector database based on the query
        With confidence threshold checking (0.6) for content relevance
        Optimized for new features like selected text processing
        """
        # Check if the query is off-topic
        if self.is_off_topic_query(query):
            # For off-topic queries, return empty results with special indicator
            return []

        # Create embedding for the query
        query_embedding = self.embedding_service.create_embedding(query)

        # Search in Qdrant for similar content
        results = self.qdrant_service.search(
            query_embedding,
            top_k=top_k
        )

        # Filter results based on confidence threshold (0.6 as specified)
        filtered_results = []
        for result in results:
            score = result.get('score', 0)
            if score >= self.confidence_threshold:
                # Add relevance score to the result for downstream use
                result['relevance_score'] = score
                # Add metadata for optimization
                result['query_similarity'] = score  # Store original similarity score
                result['relevance_rank'] = len(filtered_results) + 1  # Rank by relevance
                filtered_results.append(result)

        # For selected text queries (which often have higher priority),
        # we might want to include more context by increasing relevance
        if "selected text" in query.lower() or "regarding this selected text" in query.lower():
            # Adjust relevance scores slightly higher for selected text queries
            for result in filtered_results:
                result['relevance_score'] = min(1.0, result['relevance_score'] * 1.1)

        return filtered_results

    def validate_content_relevance(self, query: str, content_chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Validate content relevance using confidence threshold
        """
        relevant_chunks = []
        for chunk in content_chunks:
            score = chunk.get('score', 0)
            if score >= self.confidence_threshold:
                # Calculate a normalized relevance score (0-1 scale)
                normalized_score = min(score, 1.0)  # Cap at 1.0
                chunk['normalized_relevance_score'] = normalized_score
                relevant_chunks.append(chunk)

        return relevant_chunks

    def validate_content_quality(self, content: str) -> Dict[str, Any]:
        """
        Validate content quality for RAG pipeline
        """
        quality_metrics = {
            'is_valid': True,
            'issues': [],
            'quality_score': 1.0,
            'suggestions': []
        }

        # Check content length (too short content might not be useful)
        if len(content.strip()) < 20:  # At least 20 characters
            quality_metrics['is_valid'] = False
            quality_metrics['issues'].append('Content too short (< 20 characters)')
            quality_metrics['quality_score'] = 0.1
            quality_metrics['suggestions'].append('Include more detailed content')

        # Check for excessive repetition (potential quality issue)
        words = content.split()
        if len(words) > 10:  # Only check for repetition if content is substantial
            # Check if any word appears more than 20% of the time
            word_counts = {}
            for word in words:
                clean_word = word.strip('.,!?";').lower()
                word_counts[clean_word] = word_counts.get(clean_word, 0) + 1

            total_words = len(words)
            for word, count in word_counts.items():
                if len(word) > 3 and (count / total_words) > 0.2:  # Word appears in >20% of content
                    quality_metrics['issues'].append(f'Potential content repetition detected for word: {word}')
                    quality_metrics['quality_score'] *= 0.8  # Reduce quality score
                    quality_metrics['suggestions'].append(f'Reduce repetition of the word "{word}"')

        # Check for special characters that might indicate poor quality content
        special_char_ratio = sum(1 for c in content if c in '!@#$%^&*()[]{}<>?~`|\\')
        if len(content) > 0 and (special_char_ratio / len(content)) > 0.1:  # >10% special characters
            quality_metrics['issues'].append('High ratio of special characters detected')
            quality_metrics['quality_score'] *= 0.7
            quality_metrics['suggestions'].append('Reduce special characters that may indicate poor formatting')

        # Adjust quality score based on issues found
        if quality_metrics['issues']:
            quality_metrics['quality_score'] = max(0.1, quality_metrics['quality_score'])

        return quality_metrics

    def filter_low_quality_content(self, content_chunks: List[Dict[str, Any]], min_quality_score: float = 0.5) -> List[Dict[str, Any]]:
        """
        Filter out low quality content chunks
        """
        filtered_chunks = []
        for chunk in content_chunks:
            content_text = chunk.get('content', '')
            quality_result = self.validate_content_quality(content_text)

            if quality_result['quality_score'] >= min_quality_score:
                chunk['quality_score'] = quality_result['quality_score']
                filtered_chunks.append(chunk)
            else:
                print(f"Filtered out low-quality content chunk with quality score: {quality_result['quality_score']}")

        return filtered_chunks

    def construct_prompt(self, query: str, context: List[Dict[str, Any]], conversation_history: Optional[List[Dict[str, Any]]] = None) -> str:
        """
        Construct a prompt with context for the LLM
        Modified to better include conversation history in queries
        """
        # Build context string from retrieved content
        context_str = ""
        for i, item in enumerate(context):
            content = item.get('content', '')
            module = item.get('module', 'N/A')
            week = item.get('week', 'N/A')
            chapter_title = item.get('chapter_title', 'N/A')
            url = item.get('url', 'N/A')

            context_str += f"Source {i+1}:\n"
            context_str += f"Module: {module}\n"
            context_str += f"Week: {week}\n"
            context_str += f"Chapter: {chapter_title}\n"
            context_str += f"URL: {url}\n"
            context_str += f"Content: {content}\n\n"

        # Include conversation history if available with enhanced formatting
        history_str = ""
        if conversation_history:
            history_str = "Previous conversation history:\n"
            for msg in conversation_history:
                role = msg.get('role', '')
                content = msg.get('content', '')
                timestamp = msg.get('timestamp', '')

                # Format the message based on role
                if role.lower() == 'user':
                    history_str += f"Student: {content}\n"
                elif role.lower() == 'assistant':
                    history_str += f"AI Assistant: {content}\n"
                else:
                    history_str += f"{role.capitalize()}: {content}\n"

            history_str += "\n"
            history_str += "Using the previous conversation history, please continue the discussion and answer the current question based on the provided textbook content.\n\n"
        else:
            history_str = "This is the start of a new conversation. Please answer the question based on the provided textbook content.\n\n"

        # Construct the final prompt with content guardrails
        prompt = f"""You are an AI assistant for the Physical AI & Humanoid Robotics textbook. Your purpose is to help students understand the course content by answering their questions based ONLY on the provided textbook materials.

IMPORTANT CONTENT GUARDRAILS:
- You MUST ONLY use information from the provided textbook content
- You MUST NOT provide information that is not in the provided context
- You MUST NOT generate or fabricate information (hallucinate)
- You MUST cite specific chapters/modules when providing answers
- If a question cannot be answered based on the provided content, you MUST politely explain that the information is not available in the textbook
- You MUST NOT answer off-topic questions or questions about content not in the textbook
- You MUST maintain academic integrity and only provide information that exists in the provided context

{history_str}
Relevant textbook content:
{context_str}

Current question: {query}

Please provide a comprehensive answer to the current question based ONLY on the provided textbook content. Include citations to specific chapters/modules where relevant. If the question cannot be answered based on the provided content, politely explain that the information is not available in the textbook and suggest where they might find the information in the curriculum.

When referencing previous conversation history:
1. Acknowledge relevant points from the conversation history
2. Build upon previous answers if the current question is related
3. Maintain continuity with the ongoing discussion

Format your response with:
1. A clear answer to the current question based ONLY on provided content
2. Citations to the specific chapters/modules where the information was found (use the source information provided above)
3. If referencing previous conversation, acknowledge the relevant points
4. If applicable, relate the answer to the broader context of Physical AI and Humanoid Robotics
"""
        return prompt

    async def generate_response(self, query: str, conversation_history: Optional[List[Dict[str, Any]]] = None) -> Dict[str, Any]:
        """
        Generate a response using the RAG approach with proper citation formatting
        With caching for performance optimization and resource monitoring
        """
        import time as time_module

        # Track request for monitoring
        self._request_count += 1
        start_time = time_module.time()

        # Check cache first for identical queries
        cached_response = self._get_from_cache(query, conversation_history)
        if cached_response:
            self._cache_hits += 1
            processing_time = time_module.time() - start_time
            self._total_processing_time += processing_time
            return cached_response
        else:
            self._cache_misses += 1

        # Retrieve relevant content
        relevant_content = self.retrieve_relevant_content(query)

        # Check if we have sufficient relevant content for a good response
        if not relevant_content or len(relevant_content) == 0:
            # No relevant content found, provide fallback response
            fallback_response = self.generate_fallback_response(query)
            response = {
                "response": fallback_response,
                "sources": []
            }
            # Cache fallback responses too to avoid repeated processing
            self._set_in_cache(query, conversation_history, response)
            return response

        # Check if all retrieved content has low relevance scores
        avg_relevance = sum(item.get('relevance_score', 0) for item in relevant_content) / len(relevant_content)
        if avg_relevance < 0.3:  # Threshold for low-confidence query
            fallback_response = self.generate_low_confidence_fallback(query)
            response = {
                "response": fallback_response,
                "sources": []
            }
            # Cache low-confidence responses too
            self._set_in_cache(query, conversation_history, response)
            return response

        # Construct prompt with context
        prompt = self.construct_prompt(query, relevant_content, conversation_history)

        # Generate response using Google Gemini with timeout protection
        try:
            # Prepare the system message as part of the prompt since Gemini doesn't have a separate system role
            system_message = "You are an AI assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based only on the provided textbook content and maintain academic integrity. Do not make up information that is not in the provided context. If the information is not in the context, clearly state that."
            full_prompt = f"{system_message}\n\n{prompt}"

            # Use timeout-aware function for API call
            answer = await timeout_aware_gemini_call(
                model=os.getenv("MODEL_NAME", "gemini-2.5-flash"),
                prompt=full_prompt,
                timeout=45  # 45 seconds timeout for chat operations
            )

            # Format sources with proper citation formatting
            sources = []
            for item in relevant_content:
                source = {
                    "module": item.get('module', ''),
                    "week": item.get('week', ''),
                    "chapter_title": item.get('chapter_title', ''),
                    "url": item.get('url', ''),
                    "relevance_score": item.get('relevance_score', 0)
                }
                sources.append(source)

            # Apply hallucination prevention to the response
            final_response = self.prevent_hallucinations(answer, relevant_content)

            result = {
                "response": final_response,
                "sources": self.format_citations(sources)
            }

            # Cache the successful response
            self._set_in_cache(query, conversation_history, result)

            # Track processing time
            processing_time = time_module.time() - start_time
            self._total_processing_time += processing_time

            return result
        except Exception as e:
            print(f"Error generating response: {e}")
            # If there was an error with the API call, try to provide a fallback
            fallback_response = self.generate_fallback_response(query)
            response = {
                "response": fallback_response,
                "sources": []
            }
            # Cache the fallback response to prevent repeated API errors for the same query
            self._set_in_cache(query, conversation_history, response)

            # Track processing time even for errors
            processing_time = time_module.time() - start_time
            self._total_processing_time += processing_time

            return response

    def get_resource_stats(self) -> Dict[str, Any]:
        """
        Get resource usage statistics for monitoring
        """
        avg_processing_time = 0
        cache_hit_rate = 0

        if self._request_count > 0:
            avg_processing_time = self._total_processing_time / self._request_count
            total_cache_ops = self._cache_hits + self._cache_misses
            if total_cache_ops > 0:
                cache_hit_rate = self._cache_hits / total_cache_ops

        return {
            "request_count": self._request_count,
            "total_processing_time": self._total_processing_time,
            "avg_processing_time": avg_processing_time,
            "cache_hits": self._cache_hits,
            "cache_misses": self._cache_misses,
            "cache_hit_rate": cache_hit_rate,
            "cache_size": len(self._response_cache)
        }

    def generate_fallback_response(self, query: str) -> str:
        """
        Generate a fallback response when no relevant content is found
        """
        fallback_templates = [
            f"I couldn't find specific information about '{query}' in the Physical AI & Humanoid Robotics textbook. The textbook covers topics like ROS 2, NVIDIA Isaac Platform, Humanoid Kinematics, and Conversational Robotics. Could you rephrase your question or check if it relates to one of these main topics?",

            f"The information you're looking for regarding '{query}' doesn't appear to be in the current textbook content. The Physical AI & Humanoid Robotics textbook focuses on core concepts like embodied intelligence, sensor systems, robot simulation, and humanoid development. Is there a related concept I can help explain?",

            f"I'm unable to find content about '{query}' in the textbook materials. The Physical AI & Humanoid Robotics curriculum covers Weeks 1-13 with topics including Physical AI Foundations, ROS 2 Fundamentals, Robot Simulation, NVIDIA Isaac Platform, Humanoid Development, and Conversational Robotics. Would you like me to help you find information on one of these topics instead?"
        ]

        import random
        return random.choice(fallback_templates)

    def generate_low_confidence_fallback(self, query: str) -> str:
        """
        Generate a response when content relevance is low
        """
        return f"I found some content related to your question about '{query}', but the relevance is low. The information in the Physical AI & Humanoid Robotics textbook may not fully address your specific question. I recommend checking the relevant chapters on the topic or asking a more specific question related to the course material."

    def format_citations(self, sources: List[Dict[str, str]]) -> List[Dict[str, str]]:
        """
        Format citations properly for responses with standardized format and relevance scoring
        """
        formatted_sources = []
        for source in sources:
            # Create a formatted citation with consistent structure and relevance score
            formatted_source = {
                "module": source.get('module', 'N/A'),
                "week": source.get('week', 'N/A'),
                "chapter_title": source.get('chapter_title', 'N/A'),
                "url": source.get('url', ''),
                "relevance_score": source.get('relevance_score', 0),
                "relevance_label": self._get_relevance_label(source.get('relevance_score', 0)),
                "formatted_citation": self._create_formatted_citation(source)
            }
            formatted_sources.append(formatted_source)

        return formatted_sources

    def _get_relevance_label(self, score: float) -> str:
        """
        Convert numerical relevance score to human-readable label
        """
        if score >= 0.8:
            return "Highly Relevant"
        elif score >= 0.6:
            return "Moderately Relevant"
        elif score >= 0.4:
            return "Somewhat Relevant"
        else:
            return "Low Relevance"

    def _create_formatted_citation(self, source: Dict[str, str]) -> str:
        """
        Create a properly formatted citation string
        """
        module = source.get('module', 'N/A')
        week = source.get('week', 'N/A')
        chapter_title = source.get('chapter_title', 'N/A')
        url = source.get('url', '')

        # Create a consistent citation format
        if url:
            return f"{module}, {week}: \"{chapter_title}\" (See: {url})"
        else:
            return f"{module}, {week}: \"{chapter_title}\""

    def verify_content_sources(self, sources: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Verify content sources for authenticity and validity
        """
        verified_sources = []
        for source in sources:
            # Check if the source has required fields
            is_valid = True
            issues = []

            # Verify required fields exist
            if not source.get('module') or source.get('module') == 'N/A':
                is_valid = False
                issues.append('Missing or invalid module information')

            if not source.get('week') or source.get('week') == 'N/A':
                is_valid = False
                issues.append('Missing or invalid week information')

            if not source.get('chapter_title') or source.get('chapter_title') == 'N/A':
                is_valid = False
                issues.append('Missing or invalid chapter title')

            # Verify URL format if present
            url = source.get('url', '')
            if url and not self._is_valid_url(url):
                issues.append('Invalid URL format')
                is_valid = False

            # Add verification results to source
            verified_source = {**source}
            verified_source['is_verified'] = is_valid
            verified_source['verification_issues'] = issues

            verified_sources.append(verified_source)

        return verified_sources

    def _is_valid_url(self, url: str) -> bool:
        """
        Check if a URL is valid for the textbook
        """
        import re
        # Simple URL validation - check if it starts with /docs/ or has proper format
        if url.startswith('/docs/') or url.startswith('http'):
            # Basic URL pattern check
            url_pattern = re.compile(
                r'^https?://'  # http:// or https://
                r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # Domain
                r'localhost|'  # localhost
                r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # IP
                r'(?::\d+)?'  # Port
                r'(?:/?|[/?]\S+)$', re.IGNORECASE)

            # Or check for relative paths that are valid for the textbook
            relative_pattern = re.compile(r'^/docs/[\w\-_/]+$')

            return bool(re.match(url_pattern, url) or re.match(relative_pattern, url))
        return True  # Relative paths starting with /docs/ are considered valid by default

    def detect_hallucinations(self, response: str, source_content: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Detect potential hallucinations in the response by comparing with source content
        """
        hallucination_check = {
            'has_hallucinations': False,
            'confidence_score': 1.0,
            'issues': [],
            'suggestions': []
        }

        # Check if response contains information not present in source content
        source_text = " ".join([item.get('content', '') for item in source_content]).lower()
        response_lower = response.lower()

        # Look for claims in response that aren't supported by sources
        # This is a simplified check - in a production system, we'd use more sophisticated NLP techniques
        response_sentences = response.split('. ')

        for sentence in response_sentences:
            sentence_clean = re.sub(r'[^\w\s]', '', sentence.lower().strip())
            if len(sentence_clean) > 10:  # Only check sentences with substantial content
                # Check if this sentence or its key elements appear in source content
                if sentence_clean not in source_text:
                    # This might indicate a hallucination, but do a fuzzy match to be more accurate
                    words = sentence_clean.split()
                    if len(words) > 3:  # Only check sentences with multiple words
                        found_support = False
                        for word in words[:5]:  # Check first 5 words as a sample
                            if len(word) > 3 and word in source_text:  # Only check words longer than 3 chars
                                found_support = True
                                break

                        if not found_support:
                            hallucination_check['has_hallucinations'] = True
                            hallucination_check['issues'].append(f'Possible unsupported claim: "{sentence[:100]}..."')
                            hallucination_check['confidence_score'] *= 0.7  # Reduce confidence
                            hallucination_check['suggestions'].append('Verify this information is supported by the provided sources')

        # Additional checks for common hallucination patterns
        self._check_common_hallucination_patterns(response, hallucination_check)

        return hallucination_check

    def _check_common_hallucination_patterns(self, response: str, check_result: Dict[str, Any]):
        """
        Check for common hallucination patterns in the response
        """
        common_patterns = [
            (r'according to page \d+', 'References to specific page numbers not in source'),
            (r'as stated in chapter \d+', 'References to specific chapters not in source'),
            (r'experts agree that', 'Opinions not attributed to specific sources'),
            (r'scientists believe', 'Generalizations not supported by specific sources'),
            (r'source \w+ says', 'References to non-existent sources'),
            (r'recent studies show', 'References to studies not in provided content')
        ]

        for pattern, description in common_patterns:
            import re
            matches = re.findall(pattern, response, re.IGNORECASE)
            if matches:
                check_result['has_hallucinations'] = True
                check_result['issues'].extend([f'{description}: {match}' for match in matches])
                check_result['confidence_score'] *= 0.8  # Reduce confidence further
                check_result['suggestions'].append(f'Avoid making claims about {description.lower()} unless specifically in the provided sources')

    def prevent_hallucinations(self, response: str, source_content: List[Dict[str, Any]]) -> str:
        """
        Apply hallucination prevention techniques to the response
        """
        # Perform hallucination detection
        detection_result = self.detect_hallucinations(response, source_content)

        if detection_result['has_hallucinations']:
            # If hallucinations detected, modify response to be more conservative
            modified_response = response

            # Add disclaimer about sources if not already present
            if "based on the provided textbook content" not in response.lower():
                modified_response += "\n\nNote: This response is based on the provided textbook content. Some information may require verification from the original source."

            return modified_response

        return response

# Singleton instance
rag_service = RAGService()

def get_rag_service() -> RAGService:
    """
    Get the singleton RAG service instance
    """
    return rag_service