# Agent API Contract

## Agent Query Interface

### Query Request
```
POST /query
```

**Request Body**:
```json
{
  "query": "natural language question about book content",
  "scope": "full-book|section|page",
  "section_id": "optional section identifier",
  "page_id": "optional page identifier"
}
```

**Response**:
```json
{
  "content": "answer to the question based on retrieved content",
  "sources": [
    {
      "url": "source URL of the content chunk",
      "section": "section identifier",
      "page": "page identifier",
      "title": "title of source document",
      "similarity_score": "relevance score of the chunk"
    }
  ],
  "confidence": 0.85,
  "timestamp": "ISO 8601 timestamp"
}
```

### Error Responses

**400 Bad Request**: Invalid query format
**404 Not Found**: No relevant content found for the query
**500 Internal Server Error**: Service unavailable or API errors

## Retrieval Tool Interface

### Retrieve Content
```
POST /retrieve
```

**Request Body**:
```json
{
  "query": "search query for vector search",
  "top_k": 5,
  "scope": "full-book|section|page",
  "section_id": "optional section identifier",
  "threshold": 0.0
}
```

**Response**:
```json
{
  "chunks": [
    {
      "id": "chunk identifier",
      "content": "retrieved content text",
      "similarity_score": 0.92,
      "source_metadata": {
        "url": "source URL",
        "section": "section identifier",
        "page": "page identifier",
        "title": "document title"
      }
    }
  ],
  "retrieval_time_ms": 245.6
}
```

## Agent Configuration

### Environment Variables
- `OPENAI_API_KEY`: API key for OpenAI services
- `COHERE_API_KEY`: API key for Cohere embedding services
- `QDRANT_URL`: URL for Qdrant vector database
- `QDRANT_API_KEY`: API key for Qdrant access
- `COLLECTION_NAME`: Name of the Qdrant collection containing book content