# Data Model: RAG Retrieval Validation

## Overview
Data model for RAG retrieval validation and pipeline testing, defining the structure of validation results, retrieved chunks, and query parameters.

## Entities

### ValidationResult
**Description**: Captures the results of a validation run

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| validation_id | string | Yes | Unique identifier for validation run |
| timestamp | datetime | Yes | When validation was performed |
| status | string | Yes | Success/failure status of validation |
| metrics | object | Yes | Performance and accuracy metrics |
| errors | array | No | List of any errors encountered |
| query_text | string | Yes | Original query text used |
| scope | string | Yes | Scope of retrieval (full-book, page-level) |
| retrieved_count | integer | Yes | Number of chunks retrieved |
| execution_time | float | Yes | Time taken for retrieval in seconds |

**Validation Rules**:
- validation_id must be unique
- timestamp must be in ISO format
- status must be "success" or "failure"
- execution_time must be positive

### RetrievedChunk
**Description**: Represents a chunk retrieved from Qdrant during validation

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| chunk_id | string | Yes | Identifier for the retrieved chunk |
| content | string | Yes | Text content of the chunk |
| similarity_score | float | Yes | Semantic similarity score to query (0.0-1.0) |
| metadata | object | Yes | Original metadata (URL, section, chunk index) |
| embedding_vector | array | Yes | Vector representation of content |
| source_url | string | Yes | Original URL of the content |
| section | string | No | Section identifier |
| chunk_index | integer | No | Index of chunk within source |

**Validation Rules**:
- similarity_score must be between 0.0 and 1.0
- content must not be empty
- metadata must contain at least source_url
- embedding_vector must match expected dimension

### Query
**Description**: Defines parameters for retrieval validation

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| query_text | string | Yes | Original text used for retrieval |
| scope | string | Yes | Scope of retrieval (full-book, page-level) |
| top_k | integer | Yes | Number of top results to retrieve (default: 5) |
| threshold | float | No | Minimum similarity threshold (default: 0.7) |
| filters | object | No | Additional filters for retrieval |
| collection_name | string | Yes | Name of Qdrant collection to search |

**Validation Rules**:
- query_text must not be empty
- scope must be "full-book" or "page-level"
- top_k must be positive integer (1-100)
- threshold must be between 0.0 and 1.0

## Relationships

```
ValidationResult 1 ---- * RetrievedChunk
ValidationResult 1 ---- 1 Query
```

- A ValidationResult contains multiple RetrievedChunks
- A ValidationResult is associated with one Query

## State Transitions

### ValidationResult Status
```
PENDING -> VALIDATING -> [SUCCESS/FAILURE]
```

- PENDING: Validation process initiated
- VALIDATING: Retrieval and validation in progress
- SUCCESS: All validation criteria met
- FAILURE: One or more validation criteria failed

## Validation Scenarios

### Embedding Consistency Check
- Compare embedding vectors from retrieval against stored values
- Verify cosine similarity > 0.99 for same content
- Log inconsistencies with details

### Metadata Integrity Check
- Verify all metadata fields present in retrieved chunks
- Confirm URL, section, and chunk index match stored values
- Flag missing or inconsistent metadata

### Scope Validation
- For full-book scope: verify results span multiple pages/sections
- For page-level scope: verify results confined to specific URL
- Check that filters are properly applied