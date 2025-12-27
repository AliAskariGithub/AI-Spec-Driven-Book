"""
Tests for the Docusaurus RAG Ingestion Pipeline
"""
import os
import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
from pathlib import Path

# Add the backend directory to the path so we can import main
sys.path.insert(0, str(Path(__file__).parent.parent))

from main import DocusaurusRAGPipeline


class TestDocusaurusRAGPipeline(unittest.TestCase):
    @patch('main.QdrantClient')
    @patch('main.cohere.Client')
    def setUp(self, mock_cohere_client, mock_qdrant_client):
        # Set up environment variables for testing
        os.environ["COHERE_API_KEY"] = "test-key"
        os.environ["QDRANT_URL"] = "https://test-qdrant.com"
        os.environ["QDRANT_API_KEY"] = "test-qdrant-key"
        os.environ["COLLECTION_NAME"] = "test_collection"

        # Mock the clients
        self.mock_qdrant_instance = Mock()
        mock_qdrant_client.return_value = self.mock_qdrant_instance

        self.mock_cohere_instance = Mock()
        mock_cohere_client.return_value = self.mock_cohere_instance

        self.pipeline = DocusaurusRAGPipeline()

    def tearDown(self):
        # Clean up environment variables
        for key in ["COHERE_API_KEY", "QDRANT_URL", "QDRANT_API_KEY", "COLLECTION_NAME"]:
            if key in os.environ:
                del os.environ[key]

    def test_fetch_urls_from_docusaurus(self):
        with patch('main.requests.get') as mock_get:
            # Mock the response for the main page
            mock_response = Mock()
            mock_response.content = '''
            <html>
                <body>
                    <a href="/docs/intro.html">Intro</a>
                    <a href="/docs/installation.html">Installation</a>
                    <a href="/docs/usage.html">Usage</a>
                    <a href="https://other-site.com">External Link</a>
                </body>
            </html>
            '''
            mock_response.raise_for_status.return_value = None
            mock_get.return_value = mock_response

            urls = self.pipeline.fetch_urls_from_docusaurus("https://example.com")

            # Should include filtered Docusaurus URLs that match patterns (.html, /docs/, etc.)
            self.assertIn("https://example.com/docs/intro.html", urls)
            self.assertIn("https://example.com/docs/installation.html", urls)
            self.assertIn("https://example.com/docs/usage.html", urls)

            # Should not include external links
            self.assertNotIn("https://other-site.com", urls)

    def test_extract_content_from_url(self):
        with patch('main.requests.get') as mock_get:
            # Mock the response for a page
            mock_response = Mock()
            mock_response.content = '''
            <html>
                <head><title>Test Page</title></head>
                <body>
                    <main>
                        <h1>Main Title</h1>
                        <p>This is the main content of the page.</p>
                        <p>It has multiple paragraphs.</p>
                        <script>console.log('ignore this');</script>
                    </main>
                </body>
            </html>
            '''
            mock_response.raise_for_status.return_value = None
            mock_get.return_value = mock_response

            content_data = self.pipeline.extract_content_from_url("https://example.com/test")

            self.assertIsNotNone(content_data)
            self.assertEqual(content_data['url'], "https://example.com/test")
            self.assertEqual(content_data['title'], "Test Page")
            self.assertIn("main content of the page", content_data['content'])
            self.assertNotIn("console.log", content_data['content'])  # Script should be removed

    def test_chunk_text_basic(self):
        text = "This is sentence one. This is sentence two. This is sentence three."
        chunks = self.pipeline.chunk_text(text, max_tokens=20, overlap_tokens=5)

        self.assertGreater(len(chunks), 0)
        for chunk in chunks:
            self.assertIn('content', chunk)
            self.assertIn('token_count', chunk)

    def test_chunk_text_with_overlap(self):
        # Test that chunks have appropriate overlap
        long_text = "This is a sentence. " * 50  # Create a long text
        chunks = self.pipeline.chunk_text(long_text, max_tokens=15, overlap_tokens=5)

        self.assertGreater(len(chunks), 1)  # Should be split into multiple chunks

    def test_generate_embeddings(self):
        # Mock the Cohere client response
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3]] * 2  # Two embeddings
        self.mock_cohere_instance.embed.return_value = mock_response

        chunks = [
            {'content': 'Test content one', 'token_count': 10},
            {'content': 'Test content two', 'token_count': 12}
        ]

        embeddings = self.pipeline.generate_embeddings(chunks)

        self.assertEqual(len(embeddings), 2)
        for emb in embeddings:
            self.assertIn('chunk_id', emb)
            self.assertIn('content', emb)
            self.assertIn('embedding', emb)
            self.assertIn('metadata', emb)

    def test_store_embeddings(self):
        # Set up the mock for Qdrant client
        self.mock_qdrant_instance.get_collection.side_effect = Exception("Collection not found")
        self.mock_qdrant_instance.create_collection = Mock()
        self.mock_qdrant_instance.upsert = Mock()

        embeddings = [
            {
                'chunk_id': 'chunk_0',
                'content': 'Test content',
                'embedding': [0.1, 0.2, 0.3],
                'metadata': {'chunk_index': 0, 'token_count': 10, 'source_type': 'docusaurus_content'}
            }
        ]

        self.pipeline.store_embeddings(embeddings, "test_collection")

        # Verify that upsert was called
        self.mock_qdrant_instance.upsert.assert_called_once()


if __name__ == '__main__':
    unittest.main()