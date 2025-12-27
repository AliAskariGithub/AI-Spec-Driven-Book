"""
Tests for RAG agent functionality
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
import os
from pathlib import Path

# Set environment variables before importing the agent module
os.environ["OPENAI_API_KEY"] = "test-key"
os.environ["COHERE_API_KEY"] = "test-key"
os.environ["QDRANT_URL"] = "https://test-qdrant.com"
os.environ["QDRANT_API_KEY"] = "test-qdrant-key"
os.environ["COLLECTION_NAME"] = "test_collection"

# Add the backend directory to the path so we can import agent
sys.path.insert(0, str(Path(__file__).parent.parent))

from agent import RAGAgent, AgentResponse, RetrievedContentChunk, SourceReference, QueryScope, AgentConfiguration


class TestAgentDataClasses(unittest.TestCase):
    """Test cases for agent data classes"""

    def test_source_reference_creation(self):
        """Test SourceReference data class creation"""
        source = SourceReference(
            url="https://example.com",
            section="section1",
            page="page1",
            title="Example Title"
        )

        self.assertEqual(source.url, "https://example.com")
        self.assertEqual(source.section, "section1")
        self.assertEqual(source.page, "page1")
        self.assertEqual(source.title, "Example Title")

    def test_retrieved_content_chunk_creation(self):
        """Test RetrievedContentChunk data class creation"""
        source = SourceReference(url="https://example.com")
        chunk = RetrievedContentChunk(
            id="chunk1",
            content="Test content",
            similarity_score=0.85,
            source_metadata=source,
            embedding=[0.1, 0.2, 0.3]
        )

        self.assertEqual(chunk.id, "chunk1")
        self.assertEqual(chunk.content, "Test content")
        self.assertEqual(chunk.similarity_score, 0.85)
        self.assertEqual(chunk.source_metadata.url, "https://example.com")
        self.assertEqual(chunk.embedding, [0.1, 0.2, 0.3])

    def test_agent_response_creation(self):
        """Test AgentResponse data class creation"""
        source = SourceReference(url="https://example.com")
        response = AgentResponse(
            content="Test response",
            sources=[source],
            confidence=0.9
        )

        self.assertEqual(response.content, "Test response")
        self.assertEqual(len(response.sources), 1)
        self.assertEqual(response.sources[0].url, "https://example.com")
        self.assertEqual(response.confidence, 0.9)

    def test_query_scope_enum(self):
        """Test QueryScope enum values"""
        self.assertEqual(QueryScope.FULL_BOOK.value, "full-book")
        self.assertEqual(QueryScope.SECTION.value, "section-specific")
        self.assertEqual(QueryScope.PAGE.value, "page-specific")

    def test_agent_configuration_creation(self):
        """Test AgentConfiguration creation"""
        config = AgentConfiguration(
            openai_api_key="test-key",
            cohere_api_key="test-key",
            qdrant_url="https://test.qdrant.com",
            qdrant_api_key="test-key",
            collection_name="test-collection"
        )

        self.assertEqual(config.openai_api_key, "test-key")
        self.assertEqual(config.collection_name, "test-collection")


class TestRAGAgent(unittest.TestCase):
    """Test cases for RAGAgent class"""

    def setUp(self):
        """Set up test fixtures before each test method."""
        # Mock environment variables
        os.environ["OPENAI_API_KEY"] = "test-key"
        os.environ["COHERE_API_KEY"] = "test-key"
        os.environ["QDRANT_URL"] = "https://test-qdrant.com"
        os.environ["QDRANT_API_KEY"] = "test-qdrant-key"
        os.environ["COLLECTION_NAME"] = "test_collection"

    @patch('agent.QdrantClient')
    @patch('agent.cohere.Client')
    @patch('agent.OpenAI')  # Mock the OpenAI client
    def test_agent_initialization(self, mock_openai, mock_cohere, mock_qdrant):
        """Test RAGAgent initialization"""
        # Mock the clients
        self.mock_openai_instance = Mock()
        mock_openai.return_value = self.mock_openai_instance

        self.mock_cohere_instance = Mock()
        mock_cohere.return_value = self.mock_cohere_instance

        self.mock_qdrant_instance = Mock()
        mock_qdrant.return_value = self.mock_qdrant_instance

        # Create the agent (this will use the mocked clients)
        agent = RAGAgent()

        # Test that the agent was initialized with the configuration
        self.assertIsNotNone(agent.config)
        self.assertTrue(agent.config.validate())

    @patch('agent.QdrantClient')
    @patch('agent.cohere.Client')
    @patch('agent.OpenAI')  # Mock the OpenAI client
    def test_agent_query_method_exists(self, mock_openai, mock_cohere, mock_qdrant):
        """Test that the agent has a query method"""
        # Mock the clients
        self.mock_openai_instance = Mock()
        mock_openai.return_value = self.mock_openai_instance

        self.mock_cohere_instance = Mock()
        mock_cohere.return_value = self.mock_cohere_instance

        self.mock_qdrant_instance = Mock()
        mock_qdrant.return_value = self.mock_qdrant_instance

        # Create the agent (this will use the mocked clients)
        agent = RAGAgent()

        self.assertTrue(hasattr(agent, 'query'))
        self.assertTrue(callable(getattr(agent, 'query')))

    @patch('agent.retrieval_tool')
    @patch('agent.QdrantClient')
    @patch('agent.cohere.Client')
    @patch('agent.OpenAI')  # Mock the OpenAI client
    def test_agent_response_with_citations(self, mock_openai, mock_cohere, mock_qdrant, mock_retrieval):
        """Test agent response includes source citations (T019)"""
        # Mock the clients
        mock_openai_instance = Mock()
        mock_openai.return_value = mock_openai_instance

        mock_cohere_instance = Mock()
        mock_cohere.return_value = mock_cohere_instance

        mock_qdrant_instance = Mock()
        mock_qdrant.return_value = mock_qdrant_instance

        # Mock the OpenAI response
        mock_choice = Mock()
        mock_choice.message.content = "This is a test response based on the provided context."

        mock_response = Mock()
        mock_response.choices = [mock_choice]

        # Mock the OpenAI client's method
        mock_openai_instance.chat.completions.create.return_value = mock_response

        # Mock the retrieval tool to return some chunks
        mock_chunk = RetrievedContentChunk(
            id="test-id",
            content="Test content for citation",
            similarity_score=0.85,
            source_metadata=SourceReference(
                url="https://example.com",
                title="Test Document"
            )
        )
        mock_retrieval.return_value = {"chunks": [mock_chunk]}

        # Create the agent (this will use the mocked clients)
        agent = RAGAgent()

        # Ensure the agent's openai_client is the mocked instance
        agent.openai_client = mock_openai_instance

        # Call the query method
        result = agent.query("What is the test content?")

        # Verify the response has content and sources
        self.assertIsNotNone(result.content)
        self.assertGreater(len(result.sources), 0)
        self.assertEqual(result.sources[0].url, "https://example.com")

    @patch('agent.retrieval_tool')
    @patch('agent.QdrantClient')
    @patch('agent.cohere.Client')
    @patch('agent.OpenAI')  # Mock the OpenAI client
    def test_scoped_retrieval_functionality(self, mock_openai, mock_cohere, mock_qdrant, mock_retrieval):
        """Test scoped retrieval functionality (T024)"""
        # Mock the clients
        mock_openai_instance = Mock()
        mock_openai.return_value = mock_openai_instance

        mock_cohere_instance = Mock()
        mock_cohere.return_value = mock_cohere_instance

        mock_qdrant_instance = Mock()
        mock_qdrant.return_value = mock_qdrant_instance

        # Mock the OpenAI response
        mock_choice = Mock()
        mock_choice.message.content = "This is a test response for scoped retrieval."

        mock_response = Mock()
        mock_response.choices = [mock_choice]

        # Mock the OpenAI client's method
        mock_openai_instance.chat.completions.create.return_value = mock_response

        # Test full-book scope
        mock_chunk = RetrievedContentChunk(
            id="test-id",
            content="Full book content",
            similarity_score=0.85,
            source_metadata=SourceReference(
                url="https://example.com",
                title="Test Document"
            )
        )
        mock_retrieval.return_value = {"chunks": [mock_chunk]}

        # Create the agent (this will use the mocked clients)
        agent = RAGAgent()

        # Ensure the agent's openai_client is the mocked instance
        agent.openai_client = mock_openai_instance

        # Call the query method with full-book scope
        result = agent.query("What is the content?", scope="full-book")

        # Verify the call was made with correct parameters
        mock_retrieval.assert_called()
        args, kwargs = mock_retrieval.call_args
        self.assertEqual(kwargs.get('scope'), 'full-book')

    @patch('agent.retrieval_tool')
    @patch('agent.QdrantClient')
    @patch('agent.cohere.Client')
    @patch('agent.OpenAI')  # Mock the OpenAI client
    def test_full_book_vs_section_specific_query_responses(self, mock_openai, mock_cohere, mock_qdrant, mock_retrieval):
        """Test full-book vs section-specific query responses (T025)"""
        # Mock the clients
        mock_openai_instance = Mock()
        mock_openai.return_value = mock_openai_instance

        mock_cohere_instance = Mock()
        mock_cohere.return_value = mock_cohere_instance

        mock_qdrant_instance = Mock()
        mock_qdrant.return_value = mock_qdrant_instance

        # Mock the OpenAI response
        mock_choice = Mock()
        mock_choice.message.content = "This is a test response for comparison."

        mock_response = Mock()
        mock_response.choices = [mock_choice]

        # Mock the OpenAI client's method
        mock_openai_instance.chat.completions.create.return_value = mock_response

        # Test section-specific scope
        mock_chunk = RetrievedContentChunk(
            id="test-id",
            content="Section specific content",
            similarity_score=0.85,
            source_metadata=SourceReference(
                url="https://example.com/section1",
                section="section1",
                title="Test Document"
            )
        )
        mock_retrieval.return_value = {"chunks": [mock_chunk]}

        # Create the agent (this will use the mocked clients)
        agent = RAGAgent()

        # Ensure the agent's openai_client is the mocked instance
        agent.openai_client = mock_openai_instance

        # Call the query method with section-specific scope
        result = agent.query("What is the section content?", scope="section-specific", section_id="section1")

        # Verify the call was made with correct parameters
        args, kwargs = mock_retrieval.call_args
        self.assertEqual(kwargs.get('scope'), 'section-specific')
        self.assertEqual(kwargs.get('section_id'), 'section1')

    def test_source_attribution_verification(self):
        """Test comprehensive source attribution (T030)"""
        # Create a response with sources
        source = SourceReference(url="https://example.com", title="Test Document")
        response_with_sources = AgentResponse(
            content="Test response with sources",
            sources=[source],
            confidence=0.85
        )

        # Verify the response validates properly
        self.assertTrue(response_with_sources.validate_sources())

        # Create a response without sources
        response_without_sources = AgentResponse(
            content="Test response without sources",
            sources=[],
            confidence=0.0
        )

        # Verify the response without sources fails validation
        self.assertFalse(response_without_sources.validate_sources())

        # Test source attribution verification function
        from agent import verify_source_attribution
        self.assertTrue(verify_source_attribution(response_with_sources))
        self.assertFalse(verify_source_attribution(response_without_sources))

    @patch('agent.retrieval_tool')
    @patch('agent.QdrantClient')
    @patch('agent.cohere.Client')
    @patch('agent.OpenAI')  # Mock the OpenAI client
    def test_agent_behavior_when_no_relevant_content_found(self, mock_openai, mock_cohere, mock_qdrant, mock_retrieval):
        """Test agent's behavior when no relevant content is found (T031)"""
        # Mock the clients
        mock_openai_instance = Mock()
        mock_openai.return_value = mock_openai_instance

        mock_cohere_instance = Mock()
        mock_cohere.return_value = mock_cohere_instance

        mock_qdrant_instance = Mock()
        mock_qdrant.return_value = mock_qdrant_instance

        # Mock the OpenAI response
        mock_choice = Mock()
        mock_choice.message.content = "No relevant content found to answer this question."

        mock_response = Mock()
        mock_response.choices = [mock_choice]

        # Mock the OpenAI client's method
        mock_openai_instance.chat.completions.create.return_value = mock_response

        # Mock the retrieval tool to return no chunks
        mock_retrieval.return_value = {"chunks": []}

        # Create the agent (this will use the mocked clients)
        agent = RAGAgent()

        # Ensure the agent's openai_client is the mocked instance
        agent.openai_client = mock_openai_instance

        # Call the query method
        result = agent.query("What is the non-existent content?")

        # Verify the response has appropriate content for no results
        self.assertIsNotNone(result.content)
        # The response should have no sources since no content was retrieved
        self.assertEqual(len(result.sources), 0)


if __name__ == '__main__':
    unittest.main()