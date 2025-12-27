"""
Tests for RAG retrieval validation functionality
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
import os
from pathlib import Path

# Add the backend directory to the path so we can import retrieves
sys.path.insert(0, str(Path(__file__).parent.parent))

from retrieves import RAGRetrievalValidator, RetrievalScope, RetrievedChunk, ValidationResult


class TestRAGRetrievalValidator(unittest.TestCase):
    """Test cases for RAGRetrievalValidator class"""

    @patch('retrieves.QdrantClient')
    @patch('retrieves.cohere.Client')
    def setUp(self, mock_cohere_client, mock_qdrant_client):
        """Set up test fixtures before each test method."""
        # Mock environment variables
        os.environ["COHERE_API_KEY"] = "test-key"
        os.environ["QDRANT_URL"] = "https://test-qdrant.com"
        os.environ["QDRANT_API_KEY"] = "test-qdrant-key"
        os.environ["COLLECTION_NAME"] = "test_collection"

        # Create the validator (this will use the mocked clients)
        self.validator = RAGRetrievalValidator()

        # Ensure we're using the mocked clients
        self.mock_qdrant_client = mock_qdrant_client.return_value
        self.mock_cohere_client = mock_cohere_client.return_value

    def test_validate_collection_schema_success(self):
        """Test successful collection schema validation"""
        # Setup mocks
        mock_collection_info = Mock()
        mock_collection_info.config.params.vectors.size = 1024
        mock_collection_info.config.params.vectors.distance = "Cosine"

        mock_point = Mock()
        mock_point.payload = {
            'content': 'test content',
            'chunk_id': 'chunk_1',
            'source_url': 'https://example.com'
        }

        self.mock_qdrant_client.get_collection.return_value = mock_collection_info
        self.mock_qdrant_client.scroll.return_value = ([mock_point], None)

        result = self.validator.validate_collection_schema()

        self.assertTrue(result)
        self.mock_qdrant_client.get_collection.assert_called_once()

    def test_embed_query_success(self):
        """Test successful query embedding"""
        # Setup mock
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3]]
        self.mock_cohere_client.embed.return_value = mock_response

        result = self.validator.embed_query("test query")

        self.assertEqual(result, [0.1, 0.2, 0.3])
        self.mock_cohere_client.embed.assert_called_once()

    def test_validate_metadata_integrity_all_present(self):
        """Test metadata validation when all required fields are present"""
        chunks = [
            RetrievedChunk(
                chunk_id="1",
                content="test content",
                similarity_score=0.8,
                metadata={'source_url': 'https://example.com', 'chunk_index': 0},
                embedding_vector=[0.1, 0.2],
                source_url="https://example.com"
            )
        ]

        result = self.validator.validate_metadata_integrity(chunks)

        self.assertTrue(result['validation_passed'])
        self.assertEqual(result['chunks_with_url'], 1)
        self.assertEqual(result['chunks_with_content'], 1)
        self.assertEqual(result['total_chunks'], 1)

    def test_validate_metadata_integrity_missing_fields(self):
        """Test metadata validation when required fields are missing"""
        chunks = [
            RetrievedChunk(
                chunk_id="1",
                content="",
                similarity_score=0.8,
                metadata={},
                embedding_vector=[0.1, 0.2],
                source_url=""
            )
        ]

        result = self.validator.validate_metadata_integrity(chunks)

        self.assertFalse(result['validation_passed'])
        self.assertEqual(result['chunks_with_url'], 0)
        self.assertEqual(result['chunks_with_content'], 0)
        self.assertGreater(len(result['errors']), 0)

    def test_validate_embedding_consistency(self):
        """Test embedding consistency validation"""
        chunks = [
            RetrievedChunk(
                chunk_id="1",
                content="test content",
                similarity_score=0.8,
                metadata={},
                embedding_vector=[0.1, 0.2],
                source_url="https://example.com"
            )
        ]

        with patch.object(self.validator, 'embed_query', return_value=[0.1, 0.2, 0.3]):
            result = self.validator.validate_embedding_consistency("test query", chunks)

        self.assertTrue(result['query_embedding_generated'])
        self.assertEqual(result['total_checks'], 1)
        self.assertGreater(result['average_similarity'], 0)

    def test_retrieved_chunk_dataclass(self):
        """Test RetrievedChunk dataclass functionality"""
        chunk = RetrievedChunk(
            chunk_id="test_id",
            content="test content",
            similarity_score=0.85,
            metadata={'source_url': 'https://example.com'},
            embedding_vector=[0.1, 0.2, 0.3],
            source_url="https://example.com",
            section="test section",
            chunk_index=1
        )

        self.assertEqual(chunk.chunk_id, "test_id")
        self.assertEqual(chunk.content, "test content")
        self.assertEqual(chunk.similarity_score, 0.85)
        self.assertEqual(chunk.source_url, "https://example.com")

    def test_validation_result_dataclass(self):
        """Test ValidationResult dataclass functionality"""
        result = ValidationResult(
            validation_id="test_validation",
            timestamp=1234567890.0,
            status="success",
            metrics={"test": "metric"},
            errors=[],
            query_text="test query",
            scope="full-book",
            retrieved_count=5,
            execution_time=1.23
        )

        self.assertEqual(result.validation_id, "test_validation")
        self.assertEqual(result.status, "success")
        self.assertEqual(result.retrieved_count, 5)
        self.assertEqual(result.execution_time, 1.23)


class TestRetrievalScope(unittest.TestCase):
    """Test cases for RetrievalScope enum"""

    def test_retrieval_scope_values(self):
        """Test that RetrievalScope enum has correct values"""
        self.assertEqual(RetrievalScope.FULL_BOOK.value, "full-book")
        self.assertEqual(RetrievalScope.PAGE_LEVEL.value, "page-level")


if __name__ == '__main__':
    unittest.main()