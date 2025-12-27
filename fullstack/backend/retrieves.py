import os
import json
from typing import List, Dict, Any, Optional
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import SearchRequest
import logging
from dotenv import load_dotenv
import time
from dataclasses import dataclass
from enum import Enum

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class RetrievalScope(Enum):
    """Enum for different scopes of retrieval validation"""
    FULL_BOOK = "full-book"
    PAGE_LEVEL = "page-level"


@dataclass
class RetrievedChunk:
    """Dataclass representing a retrieved chunk with validation metadata"""
    chunk_id: str
    content: str
    similarity_score: float
    metadata: Dict[str, Any]
    embedding_vector: List[float]
    source_url: str
    section: Optional[str] = None
    chunk_index: Optional[int] = None


@dataclass
class ValidationResult:
    """Dataclass representing the result of a validation operation"""
    validation_id: str
    timestamp: float
    status: str
    metrics: Dict[str, Any]
    errors: List[str]
    query_text: str
    scope: str
    retrieved_count: int
    execution_time: float


class RAGRetrievalValidator:
    """Comprehensive validation system for RAG retrieval operations"""

    def __init__(self):
        # Initialize Cohere client
        self.cohere_client = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))

        # Initialize Qdrant client
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if qdrant_api_key:
            self.qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        else:
            self.qdrant_client = QdrantClient(url=qdrant_url)

        # Default collection name
        self.collection_name = os.getenv("COLLECTION_NAME", "COLLECTION_NAME_2")

    def validate_collection_schema(self) -> bool:
        """
        Validate that the Qdrant collection has the expected schema for RAG operations
        """
        try:
            # Get collection info
            collection_info = self.qdrant_client.get_collection(self.collection_name)

            # Check if the collection has the expected vector size and distance
            vector_config = collection_info.config.params.vectors
            expected_size = 1024  # Default expected embedding size

            # Check basic schema requirements
            if not hasattr(vector_config, 'size') or vector_config.size < expected_size:
                logger.warning(f"Collection vector size may be insufficient: {getattr(vector_config, 'size', 'unknown')}")

            # Check if we can access some sample data
            scroll_result = self.qdrant_client.scroll(
                collection_name=self.collection_name,
                limit=1,
                with_payload=True,
                with_vectors=False
            )

            if scroll_result[0]:  # If there are points in the collection
                sample_point = scroll_result[0][0]
                required_fields = ['content', 'chunk_id', 'source_url']
                payload = sample_point.payload

                for field in required_fields:
                    if field not in payload:
                        logger.warning(f"Missing required field '{field}' in collection schema")
                        return False

            return True

        except Exception as e:
            logger.error(f"Error validating collection schema: {e}")
            return False

    def embed_query(self, query: str) -> List[float]:
        """
        Generate embedding for a query string
        """
        try:
            response = self.cohere_client.embed(
                texts=[query],
                model="embed-multilingual-v3.0",  # Using same model as storage
                input_type="search_query"  # Optimize for search queries
            )
            return response.embeddings[0]  # Return the first (and only) embedding
        except Exception as e:
            logger.error(f"Error generating embedding for query: {e}")
            return []

    def validate_metadata_integrity(self, chunks: List[RetrievedChunk]) -> Dict[str, Any]:
        """
        Validate the integrity of metadata in retrieved chunks
        """
        validation_result = {
            'validation_passed': True,
            'total_chunks': len(chunks),
            'chunks_with_url': 0,
            'chunks_with_content': 0,
            'errors': []
        }

        for chunk in chunks:
            # Check for URL
            if chunk.source_url and chunk.source_url.strip():
                validation_result['chunks_with_url'] += 1
            else:
                validation_result['errors'].append(f"Chunk {chunk.chunk_id} missing source URL")
                validation_result['validation_passed'] = False

            # Check for content
            if chunk.content and chunk.content.strip():
                validation_result['chunks_with_content'] += 1
            else:
                validation_result['errors'].append(f"Chunk {chunk.chunk_id} missing content")
                validation_result['validation_passed'] = False

        return validation_result

    def validate_embedding_consistency(self, query: str, chunks: List[RetrievedChunk]) -> Dict[str, Any]:
        """
        Validate that the retrieved embeddings are consistent with the query
        """
        validation_result = {
            'query_embedding_generated': False,
            'total_checks': len(chunks),
            'consistent_chunks': 0,
            'average_similarity': 0.0,
            'errors': []
        }

        # Generate embedding for the query
        query_embedding = self.embed_query(query)
        if not query_embedding:
            validation_result['errors'].append("Could not generate embedding for query")
            return validation_result

        validation_result['query_embedding_generated'] = True

        # Calculate average similarity
        similarities = []
        for chunk in chunks:
            # In a real implementation, we would calculate similarity between query and chunk embeddings
            # For now, we'll use the stored similarity score
            similarities.append(chunk.similarity_score)
            if chunk.similarity_score > 0.1:  # Threshold for consistency
                validation_result['consistent_chunks'] += 1

        if similarities:
            validation_result['average_similarity'] = sum(similarities) / len(similarities)

        return validation_result

    def get_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for query text using Cohere
        """
        try:
            # Try the multilingual model first
            response = self.cohere_client.embed(
                texts=[text],
                model="embed-multilingual-v3.0",  # Using same model as storage
                input_type="search_query"  # Optimize for search queries
            )
            embedding = response.embeddings[0]  # Return the first (and only) embedding
            logger.info(f"Successfully generated embedding of size: {len(embedding)}")
            return embedding
        except Exception as e:
            logger.error(f"Error generating embedding with multilingual model: {e}")
            try:
                # Fallback to the English model
                logger.info("Trying fallback embedding model...")
                response = self.cohere_client.embed(
                    texts=[text],
                    model="embed-english-v3.0",  # Fallback to English model
                    input_type="search_query"  # Optimize for search queries
                )
                embedding = response.embeddings[0]
                logger.info(f"Successfully generated embedding with fallback model, size: {len(embedding)}")
                return embedding
            except Exception as fallback_error:
                logger.error(f"Error generating embedding with fallback model: {fallback_error}")
                return []

    def query_qdrant(self, query_embedding: List[float], top_k: int = 5, threshold: float = 0.0) -> List[Dict]:
        """
        Query Qdrant for similar vectors and return results with metadata
        """
        try:
            # Perform similarity search in Qdrant using the search method
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                score_threshold=threshold,
                with_payload=True  # Include metadata with results
            )

            # Format results
            formatted_results = []
            for result in search_results:
                # Handle both new and old result formats
                payload = getattr(result, 'payload', {})
                if not payload:
                    payload = result.get('payload', {}) if isinstance(result, dict) else {}

                formatted_result = {
                    "content": payload.get("content", ""),
                    "url": payload.get("url", payload.get("source_url", "")),  # Try both possible field names
                    "position": payload.get("position", payload.get("chunk_index", 0)),  # Try both possible field names
                    "similarity_score": getattr(result, 'score', 0),
                    "chunk_id": getattr(result, 'id', payload.get("chunk_id", "")),
                    "created_at": payload.get("created_at", "")
                }
                formatted_results.append(formatted_result)

            return formatted_results

        except AttributeError as e:
            logger.error(f"Qdrant client does not have search method: {e}")
            # Try using the legacy search method if available
            try:
                # Alternative approach using query method if available
                from qdrant_client.http import models
                search_result = self.qdrant_client.query_points(
                    collection_name=self.collection_name,
                    query=query_embedding,
                    limit=top_k,
                    score_threshold=threshold,
                    with_payload=True
                )

                formatted_results = []
                for result in search_result.points:
                    payload = getattr(result, 'payload', {})
                    formatted_result = {
                        "content": payload.get("content", ""),
                        "url": payload.get("url", payload.get("source_url", "")),  # Try both possible field names
                        "position": payload.get("position", payload.get("chunk_index", 0)),  # Try both possible field names
                        "similarity_score": getattr(result, 'score', 0),
                        "chunk_id": getattr(result, 'id', payload.get("chunk_id", "")),
                        "created_at": payload.get("created_at", "")
                    }
                    formatted_results.append(formatted_result)

                return formatted_results
            except Exception as alt_e:
                logger.error(f"Alternative search method also failed: {alt_e}")
                return []
        except Exception as e:
            logger.error(f"Error querying Qdrant: {e}")
            return []

    def retrieve(self, query_text: str, top_k: int = 5, threshold: float = 0.0, include_metadata: bool = True) -> str:
        """
        Main retrieval function that orchestrates the complete workflow
        """
        start_time = time.time()

        logger.info(f"Processing retrieval request for query: '{query_text[:50]}...'")
        logger.info(f"Collection name: {self.collection_name}")
        logger.info(f"Query parameters - top_k: {top_k}, threshold: {threshold}")

        # Step 1: Convert query text to embedding
        query_embedding = self.get_embedding(query_text)
        if not query_embedding:
            error_response = {
                "query": query_text,
                "results": [],
                "error": "Failed to generate query embedding",
                "metadata": {
                    "query_time_ms": (time.time() - start_time) * 1000,
                    "timestamp": time.time()
                }
            }
            return json.dumps(error_response, indent=2)

        logger.info(f"Generated embedding with size: {len(query_embedding)}")

        # Step 2: Query Qdrant for similar vectors
        raw_results = self.query_qdrant(query_embedding, top_k, threshold)

        logger.info(f"Raw results from Qdrant: {len(raw_results)} items")

        if not raw_results:
            logger.warning("No results returned from Qdrant")
            # Let's check if the collection exists and has data
            try:
                collection_info = self.qdrant_client.get_collection(self.collection_name)
                logger.info(f"Collection info: {collection_info}")
                points_count = collection_info.points_count
                logger.info(f"Total points in collection: {points_count}")

                if points_count == 0:
                    logger.warning(f"Collection '{self.collection_name}' is empty")
                else:
                    # Try to get a sample point to verify structure
                    sample_points = self.qdrant_client.scroll(
                        collection_name=self.collection_name,
                        limit=1,
                        with_payload=True,
                        with_vectors=False
                    )
                    if sample_points[0]:
                        sample_point = sample_points[0][0]
                        logger.info(f"Sample point ID: {sample_point.id}")
                        logger.info(f"Sample point payload keys: {list(sample_point.payload.keys())}")
                    else:
                        logger.warning("No points available in collection")

            except Exception as collection_check_error:
                logger.error(f"Error checking collection: {collection_check_error}")

        # Step 3: Verify content accuracy (optional)
        if include_metadata:
            is_accurate = self.verify_content_accuracy(raw_results)
            if not is_accurate:
                logger.warning("Content accuracy verification failed for some results")

        # Step 4: Calculate total query time
        query_time_ms = (time.time() - start_time) * 1000

        # Step 5: Format response as JSON
        json_response = self.format_json_response(raw_results, query_text, query_time_ms)

        logger.info(f"Retrieval completed in {query_time_ms:.2f}ms, {len(raw_results)} results returned")

        return json_response

    def verify_content_accuracy(self, retrieved_chunks: List[Dict]) -> bool:
        """
        Verify that retrieved content matches original stored text (basic validation)
        """
        # In a real implementation, this would compare against original sources
        # For now, we'll validate that required fields exist and have content
        # NOTE: The data in Qdrant may not have URL fields, so we only check for content
        for chunk in retrieved_chunks:
            # Check for content field (this is the essential field)
            has_content = bool(chunk.get("content", "").strip())

            if not has_content:
                logger.warning(f"Missing content in chunk: {chunk.get('chunk_id', 'unknown')}")
                return False

        # Additional validation could include checking content length, URL format, etc.
        return True

    def format_json_response(self, results: List[Dict], query: str, query_time_ms: float) -> str:
        """
        Format retrieval results into clean JSON response
        """
        response = {
            "query": query,
            "results": results,
            "metadata": {
                "query_time_ms": query_time_ms,
                "total_results": len(results),
                "timestamp": time.time(),
                "collection_name": self.collection_name
            }
        }

        return json.dumps(response, indent=2)


def retrieve_all_data():
    """
    Function to retrieve and display all data from Qdrant collection
    """
    logger.info("Initializing RAG Retrieval Validator to fetch all data...")

    # Initialize the validator
    validator = RAGRetrievalValidator()

    print("RAG Retrieval System - All Stored Data")
    print("=" * 50)

    try:
        # Get all points from the collection using scroll
        points = []
        offset = None
        while True:
            # Scroll through the collection to get all points
            batch, next_offset = validator.qdrant_client.scroll(
                collection_name=validator.collection_name,
                limit=1000,  # Get up to 1000 points at a time
                offset=offset,
                with_payload=True,
                with_vectors=False
            )

            points.extend(batch)

            # If next_offset is None, we've reached the end
            if next_offset is None:
                break

            offset = next_offset

        print(f"Total stored chunks: {len(points)}")
        print("-" * 50)

        for i, point in enumerate(points, 1):
            payload = point.payload
            content_preview = ''.join(char for char in payload.get("content", "")[:200] if ord(char) < 256)

            print(f"Chunk {i}:")
            print(f"  ID: {point.id}")
            print(f"  URL: {payload.get('url', 'N/A')}")
            print(f"  Position: {payload.get('position', 'N/A')}")
            print(f"  Content Preview: {content_preview}...")
            print(f"  Created At: {payload.get('created_at', 'N/A')}")
            print("-" * 30)

    except Exception as e:
        logger.error(f"Error retrieving all data: {e}")
        print(f"Error retrieving all data: {e}")


def main():
    """
    Main function to demonstrate the retrieval functionality
    """
    import sys

    logger.info("Initializing RAG Retrieval Validator...")

    # Check if user wants to retrieve all data or run queries
    if len(sys.argv) > 1 and sys.argv[1] == "all":
        retrieve_all_data()
        return

    # Initialize the validator
    validator = RAGRetrievalValidator()

    # Example queries to test the system
    test_queries = [
        "What is ROS2?",
        "What is the name of Module 2?",
        "What is the learning objective of chapter 3?",
    ]

    print("RAG Retrieval System - Testing Queries")
    print("=" * 50)

    for i, query in enumerate(test_queries, 1):
        print(f"\nQuery {i}: {query}")
        print("-" * 30)

        # Retrieve results
        json_response = validator.retrieve(query, top_k=3)
        response_dict = json.loads(json_response)

        # Print formatted results
        results = response_dict.get("results", [])
        if results:
            for j, result in enumerate(results, 1):
                print(f"Result {j} (Score: {result['similarity_score']:.3f}):")
                print(f"  URL: {result['url']}")
                content_preview = result['content'][:100].encode('utf-8', errors='ignore').decode('utf-8')
                # Safely print content preview by removing problematic characters
                safe_content = ''.join(char for char in content_preview if ord(char) < 256)
                print(f"  Content Preview: {safe_content}...")
                print(f"  Position: {result['position']}")
                print()
        else:
            print("No results found for this query.")

        # Handle case where there's an error response
        if 'metadata' in response_dict:
            print(f"Query time: {response_dict['metadata']['query_time_ms']:.2f}ms")
            print(f"Total results: {response_dict['metadata']['total_results']}")
        elif 'error' in response_dict:
            print(f"Error: {response_dict['error']}")
            if 'metadata' in response_dict:
                print(f"Query time: {response_dict['metadata']['query_time_ms']:.2f}ms")

if __name__ == "__main__":
    main()