# API Contract: RAG Ingestion Pipeline

## Overview
This document defines the API contract for the RAG ingestion pipeline service that extracts content from Docusaurus sites, generates embeddings, and stores them in Qdrant.

## Service Interface

### Content Extraction Service
```python
class ContentExtractor:
    def extract_from_docusaurus(self, url: str) -> List[ContentDocument]:
        """
        Extract content from a Docusaurus website URL

        Args:
            url: URL of the Docusaurus website to extract content from

        Returns:
            List of ContentDocument objects containing extracted content and metadata

        Raises:
            ExtractionError: If content cannot be extracted from the URL
            InvalidURLError: If the URL is not a valid Docusaurus site
        """
        pass
```

### Text Chunker Service
```python
class TextChunker:
    def chunk_content(self,
                     content_documents: List[ContentDocument],
                     max_tokens: int = 512,
                     overlap_tokens: int = 64) -> List[Chunk]:
        """
        Split content documents into token-aware chunks with overlap

        Args:
            content_documents: List of content documents to chunk
            max_tokens: Maximum number of tokens per chunk
            overlap_tokens: Number of overlapping tokens between adjacent chunks

        Returns:
            List of Chunk objects with token-aware boundaries

        Raises:
            ChunkingError: If chunking cannot be performed
        """
        pass
```

### Embedding Generator Service
```python
class EmbeddingGenerator:
    def generate_embeddings(self, chunks: List[Chunk], model: str = "embed-english-v3.0") -> List[EmbeddingVector]:
        """
        Generate embeddings for content chunks using Cohere models

        Args:
            chunks: List of text chunks to generate embeddings for
            model: Cohere embedding model to use

        Returns:
            List of EmbeddingVector objects with vector representations

        Raises:
            EmbeddingError: If embeddings cannot be generated
        """
        pass
```

### Vector Storage Service
```python
class VectorStorage:
    def store_embeddings(self,
                        embeddings: List[EmbeddingVector],
                        collection_name: str) -> bool:
        """
        Store embeddings in Qdrant vector database

        Args:
            embeddings: List of embedding vectors to store
            collection_name: Name of the Qdrant collection to store in

        Returns:
            True if storage was successful, False otherwise

        Raises:
            StorageError: If embeddings cannot be stored
        """
        pass
```

## Data Contracts

### ContentDocument
```python
class ContentDocument:
    id: str
    url: str
    title: str
    content: str
    headings_hierarchy: Dict
    created_at: datetime
    updated_at: datetime
```

### Chunk
```python
class Chunk:
    id: str
    document_id: str
    content: str
    chunk_index: int
    token_count: int
    overlap_with_next: str
    metadata: Dict
```

### EmbeddingVector
```python
class EmbeddingVector:
    id: str
    chunk_id: str
    vector: List[float]
    vector_size: int
    created_at: datetime
    model_used: str
```

## Error Contracts

### Standard Error Response
```python
class APIError:
    error_code: str
    message: str
    details: Optional[Dict]
    timestamp: datetime
```

### Specific Error Types
- `EXTRACTION_ERROR`: Content extraction failed
- `CHUNKING_ERROR`: Text chunking failed
- `EMBEDDING_ERROR`: Embedding generation failed
- `STORAGE_ERROR`: Vector storage failed
- `INVALID_URL_ERROR`: Invalid Docusaurus URL provided
- `RATE_LIMIT_ERROR`: External API rate limit exceeded