# Docusaurus RAG Ingestion Pipeline

A Python-based pipeline for extracting content from Docusaurus-based book websites, generating embeddings, and storing them in Qdrant Cloud for RAG applications.

## Features

- Extracts content from Docusaurus websites
- Token-aware content chunking with overlap
- Generates embeddings using Cohere models
- Stores vectors in Qdrant Cloud with metadata
- Single file implementation for simplicity
- Configurable via environment variables

## Prerequisites

- Python 3.11+
- uv package manager
- Access to Cohere API
- Access to Qdrant Cloud

## Setup

1. Clone the repository
2. Navigate to the backend directory:
   ```bash
   cd fullstack/backend
   ```

3. Install dependencies using uv:
   ```bash
   uv sync
   ```

4. Copy and configure environment variables:
   ```bash
   cp .env.example .env
   ```

   Edit `.env` with your API keys and configuration.

## Usage

Run the complete pipeline:
```bash
uv run python main.py --url https://your-docusaurus-site.com
```

With custom parameters:
```bash
uv run python main.py --url https://your-docusaurus-site.com --chunk-size 512 --overlap 64 --collection my_collection
```

## Configuration

### Environment Variables

- `COHERE_API_KEY`: Your Cohere API key
- `QDRANT_URL`: Your Qdrant Cloud URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `COLLECTION_NAME`: Qdrant collection name (default: ragchatbot-embedding)
- `CHUNK_SIZE_TOKENS`: Maximum tokens per chunk (default: 512)
- `CHUNK_OVERLAP_TOKENS`: Token overlap between chunks (default: 64)
- `COHERE_MODEL`: Cohere model to use (default: embed-english-v3.0)

### Command Line Arguments

- `--url`: Required Docusaurus site URL to process
- `--chunk-size`: Maximum tokens per chunk (default: 512)
- `--overlap`: Token overlap between chunks (default: 64)
- `--collection`: Qdrant collection name (default: ragchatbot-embedding)

## Architecture

The pipeline consists of these main steps:

1. **URL Extraction**: Discovers all pages from the Docusaurus site
2. **Content Extraction**: Extracts clean text content from each page
3. **Content Chunking**: Splits content into token-aware chunks with overlap
4. **Embedding Generation**: Creates vector embeddings using Cohere
5. **Vector Storage**: Stores embeddings in Qdrant Cloud with metadata

## Project Structure

- `main.py`: Complete pipeline in a single file
- `pyproject.toml`: Project dependencies and configuration
- `.env.example`: Example environment variables file
- `README.md`: This file