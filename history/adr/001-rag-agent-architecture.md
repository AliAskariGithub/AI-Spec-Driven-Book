# ADR 001: RAG-Enabled Agent Architecture

## Status
Accepted

## Date
2025-12-26

## Context
We need to build a RAG-enabled agent that answers questions using retrieved book content only, with proper source citations and deterministic behavior. The agent should integrate Qdrant-based retrieval as a callable tool and ensure grounded responses.

## Decision
We chose the following architectural approach:

1. **Single-file implementation**: All agent functionality in `agent.py` for simplicity and maintainability.

2. **OpenAI Agent SDK integration**: Using OpenAI's SDK to create the agent with system instructions that enforce tool-first behavior for knowledge-based queries.

3. **Cohere embeddings**: Using Cohere's embedding model (`embed-english-v3.0`) for generating query embeddings, matching the ingestion pipeline.

4. **Qdrant vector database**: Using Qdrant Cloud as the vector database for storing and retrieving content chunks with metadata.

5. **Data class structure**: Defined clear data classes for:
   - `SourceReference`: Contains URL, section, page, and title information
   - `RetrievedContentChunk`: Contains content, similarity scores, and source metadata
   - `AgentResponse`: Contains response content, sources, and confidence scores
   - `QueryScope`: Enum for different retrieval scopes (full-book, section-specific, page-specific)

6. **Tool-based retrieval**: Created a `retrieval_tool` function that queries Qdrant using Cohere embeddings, with scope-based filtering capabilities.

7. **Deterministic behavior**: Set OpenAI temperature to 0.0 for consistent responses to identical queries.

8. **Source attribution**: Enforced that all responses include proper source citations with URL references.

9. **Error handling**: Implemented proper error handling for Qdrant unavailability and fallback responses when no relevant content is found.

10. **Environment-based configuration**: Used `AgentConfiguration` class to handle environment variables with validation.

## Alternatives Considered

1. **Different embedding models**: Considered OpenAI embeddings vs Cohere embeddings. Chose Cohere to match the ingestion pipeline.

2. **Different vector databases**: Considered Pinecone, Weaviate, and FAISS. Chose Qdrant for its robust filtering and cloud capabilities.

3. **Multi-file vs single-file**: Considered splitting functionality across multiple files. Chose single-file for simplicity in the initial implementation.

4. **Different LLM providers**: Considered Anthropic Claude vs OpenAI. Chose OpenAI for better integration with the OpenAI Agent SDK.

## Consequences

### Positive
- Simple, maintainable codebase in a single file
- Consistent behavior with deterministic responses
- Proper source attribution ensuring trustworthiness
- Scalable vector search with Qdrant
- Clear separation of concerns with data classes
- Comprehensive error handling

### Negative
- Single file may become large as features are added
- Tied to specific vendor APIs (OpenAI, Cohere, Qdrant)
- Requires multiple API keys and external dependencies

## Technical Details

The agent follows this flow:
1. User submits a query with optional scope parameters
2. Query is passed to the retrieval tool which searches Qdrant using Cohere embeddings
3. Retrieved content chunks are formatted and passed to OpenAI LLM
4. LLM generates response based only on provided context
5. Response includes source citations and confidence score