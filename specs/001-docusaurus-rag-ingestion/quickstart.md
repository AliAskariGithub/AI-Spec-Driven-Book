# Quickstart: Docusaurus RAG Ingestion Pipeline

## Prerequisites
- Python 3.11 or higher
- Git
- Access to Cohere API key
- Access to Qdrant Cloud cluster
- uv package manager installed

## Setup

### 1. Clone the repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Navigate to backend directory
```bash
cd fullstack/backend
```

### 3. Install uv (if not already installed)
```bash
pip install uv
```

### 4. Install dependencies using uv
```bash
uv sync
```

### 5. Configure environment variables
Copy the example environment file:
```bash
cp .env.example .env
```

Edit `.env` and set the required values:
```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
DOCS_URL=https://your-docusaurus-site.com
COLLECTION_NAME=book_content
```

## Running the Pipeline

### Run the full ingestion pipeline:
```bash
uv run python main.py --url https://your-docusaurus-site.com
```

### Run with specific parameters:
```bash
uv run python main.py --url https://your-docusaurus-site.com --chunk-size 512 --overlap 64
```

## Configuration Options

### Environment Variables
- `COHERE_API_KEY`: API key for Cohere embedding service
- `QDRANT_URL`: URL for Qdrant Cloud cluster
- `QDRANT_API_KEY`: API key for Qdrant Cloud
- `CHUNK_SIZE_TOKENS`: Maximum tokens per chunk (default: 512)
- `CHUNK_OVERLAP_TOKENS`: Token overlap between chunks (default: 64)
- `COHERE_MODEL`: Cohere model to use (default: embed-english-v3.0)
- `COLLECTION_NAME`: Name of Qdrant collection (default: documents)

### Command Line Arguments
- `--url`: Docusaurus site URL to crawl
- `--chunk-size`: Maximum tokens per chunk (default: 512)
- `--overlap`: Token overlap between chunks (default: 64)
- `--collection`: Qdrant collection name
- `--batch-size`: Number of items to process in each batch (default: 10)

## Verification

After running the pipeline, verify the results:
1. Check that content was extracted from the Docusaurus site
2. Confirm embeddings were generated successfully
3. Verify vectors were stored in Qdrant Cloud
4. Test retrieval of stored content