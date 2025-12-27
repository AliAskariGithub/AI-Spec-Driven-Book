# Quickstart: RAG Retrieval Validation

## Overview
Quickstart guide for implementing and running RAG retrieval validation and pipeline testing.

## Prerequisites
- Python 3.11+
- Access to Qdrant Cloud instance
- Cohere API key
- Existing vector data in Qdrant collection from ingestion pipeline

## Setup

### 1. Environment Configuration
Create/update your `.env` file in the backend directory:
```bash
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
COLLECTION_NAME=ragchatbot-embedding
```

### 2. Install Dependencies
The required dependencies should already be installed from the ingestion pipeline:
```bash
cd fullstack/backend
uv sync  # or pip install -r requirements.txt if using pip
```

## Implementation

### 1. Create the Validation Script
Create `retrieves.py` in the `fullstack/backend` directory with the following functionality:
- Qdrant Cloud connection using environment configuration
- Cohere embedding generation using the same model as ingestion
- Semantic search with configurable parameters
- Metadata integrity validation
- Scope-based retrieval (full-book and page-level)

### 2. Run Validation
Execute the validation script:
```bash
cd fullstack/backend
python retrieves.py --query "your test query here" --scope full-book --top-k 5
```

## Basic Usage Examples

### Validate Full-book Retrieval
```bash
python retrieves.py --query "What are the key principles of RAG systems?" --scope full-book --top-k 5
```

### Validate Page-level Retrieval
```bash
python retrieves.py --query "Explain semantic search" --scope page-level --url-filter "https://example.com/docs/intro" --top-k 3
```

### Run Comprehensive Pipeline Validation
```bash
python retrieves.py --validate-pipeline
```

## Configuration Options
- `--query`: Test query text for retrieval
- `--scope`: Retrieval scope (full-book, page-level)
- `--top-k`: Number of results to retrieve (default: 5)
- `--threshold`: Minimum similarity threshold (default: 0.7)
- `--url-filter`: Filter results by specific URL (for page-level scope)
- `--collection`: Qdrant collection name (default: from environment)
- `--validate-pipeline`: Run comprehensive pipeline validation

## Expected Output
The validation script will output:
- Retrieved chunks with similarity scores
- Metadata validation results
- Performance metrics (retrieval time, etc.)
- Validation status (success/failure)
- Any errors encountered during validation

## Next Steps
1. Implement the `retrieves.py` script with the planned functionality
2. Write unit tests for validation functions
3. Test with various query types and scopes
4. Validate against existing ingested data