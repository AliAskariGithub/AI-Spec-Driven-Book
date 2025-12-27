"""
Test script to verify the Docusaurus RAG pipeline with a sample site
"""
import os
from main import main as pipeline_main

def test_with_sample_site():
    """Test the pipeline with a sample Docusaurus site"""
    print("Testing Docusaurus RAG pipeline with sample site...")

    # Set up environment variables (these would need to be real for actual test)
    os.environ["COHERE_API_KEY"] = os.getenv("COHERE_API_KEY", "COHERE_API_KEY_BACKUP")
    os.environ["QDRANT_URL"] = os.getenv("QDRANT_URL", "https://test-qdrant.com")
    os.environ["QDRANT_API_KEY"] = os.getenv("QDRANT_API_KEY", "test-key")

    # Use a sample Docusaurus documentation site for testing
    test_url = "https://docusaurus.io/docs"  # Example Docusaurus site

    print(f"Testing with URL: {test_url}")
    print("Note: This test requires valid API keys in environment variables")

    # For now, just show the command that would be run
    print(f"\nTo run the pipeline manually:")
    print(f"cd fullstack/backend")
    print(f"uv run python main.py --url {test_url} --collection test_collection")

    # Check if API keys are set
    if not os.getenv("COHERE_API_KEY") or os.getenv("COHERE_API_KEY") == "YOUR_COHERE_API_KEY_HERE":
        print("\n⚠️  WARNING: COHERE_API_KEY not set. Pipeline will fail without valid API key.")
    if not os.getenv("QDRANT_URL") or os.getenv("QDRANT_URL") == "YOUR_QDRANT_URL_HERE":
        print("\n⚠️  WARNING: QDRANT_URL not set. Pipeline will fail without valid Qdrant URL.")
    if not os.getenv("QDRANT_API_KEY"):
        print("\n⚠️  WARNING: QDRANT_API_KEY not set. Pipeline will fail without valid Qdrant API key.")

    print("\nTest script completed. Set environment variables and run manually to test full pipeline.")

if __name__ == "__main__":
    test_with_sample_site()