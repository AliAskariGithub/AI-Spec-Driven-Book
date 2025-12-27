# Data Model: RAG-Enabled Agent

## Core Entities

### Agent Response
- **Fields**:
  - content: string (the actual response text)
  - sources: array of SourceReference (citations to original content)
  - confidence: float (confidence level in response accuracy)
  - timestamp: datetime (when response was generated)
- **Validation**: Must include at least one source reference for any factual claim
- **Relationships**: Connected to multiple Retrieved Content Chunks

### Retrieved Content Chunk
- **Fields**:
  - id: string (unique identifier for the chunk)
  - content: string (the actual content text)
  - similarity_score: float (relevance score from vector search)
  - source_metadata: SourceReference (where this content came from)
  - embedding: array of float (vector representation)
- **Validation**: Content must be non-empty and have valid source metadata
- **Relationships**: Belongs to Agent Response

### Source Reference
- **Fields**:
  - url: string (original location of content)
  - section: string (specific section identifier)
  - page: string (page identifier if applicable)
  - title: string (title of source document)
- **Validation**: Must include at least a URL or other identifying information
- **Relationships**: Connected to Retrieved Content Chunk

### Query Scope
- **Fields**:
  - type: enum (full-book, section-specific, page-specific)
  - identifier: string (optional specific identifier for section/page)
  - description: string (human-readable description of scope)
- **Validation**: Type must be one of the defined enum values
- **Relationships**: Used by Agent to determine retrieval strategy

## State Transitions

### Agent Response Generation
1. Query received → Query parsed for scope
2. Query parsed → Retrieval tool called with appropriate scope
3. Retrieval completed → Content chunks validated for relevance
4. Content validated → Response drafted with citations
5. Response drafted → Response returned with source references

## Validation Rules

- All agent responses must contain source citations for factual claims
- Retrieved content chunks must have non-empty content and valid metadata
- Query scope must be properly identified and validated before retrieval
- Agent responses must be grounded in retrieved content only