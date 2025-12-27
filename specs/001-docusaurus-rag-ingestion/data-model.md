# Data Model: Docusaurus RAG Ingestion Pipeline

## Content Document
Represents extracted text content from Docusaurus pages, including metadata like URL, title, headings hierarchy, and source location.

**Fields**:
- `id`: Unique identifier for the document
- `url`: Source URL of the Docusaurus page
- `title`: Page title extracted from HTML
- `content`: Clean text content extracted from the page
- `headings_hierarchy`: Structured representation of headings and sections
- `created_at`: Timestamp when document was extracted
- `updated_at`: Timestamp when document was last processed

**Validation rules**:
- URL must be a valid Docusaurus page URL
- Content must not be empty
- Created_at must be a valid timestamp

## Chunk
Subsection of a document that respects token boundaries while maintaining contextual coherence.

**Fields**:
- `id`: Unique identifier for the chunk
- `document_id`: Reference to parent Content Document
- `content`: Text content of the chunk
- `chunk_index`: Position of this chunk in the original document
- `token_count`: Number of tokens in the chunk
- `overlap_with_next`: Content overlap with the next chunk (if applicable)
- `metadata`: Additional metadata about the chunk

**Validation rules**:
- Content must not exceed maximum token limit
- Chunk_index must be a non-negative integer
- Token_count must be consistent with actual tokenization

## Embedding Vector
Numeric representation of text content with semantic meaning, stored with associated metadata for retrieval.

**Fields**:
- `id`: Unique identifier for the embedding
- `chunk_id`: Reference to the source Chunk
- `vector`: Numeric vector representation (list of floats)
- `vector_size`: Dimension of the embedding vector
- `created_at`: Timestamp when embedding was generated
- `model_used`: Name/version of the embedding model used

**Validation rules**:
- Vector must have consistent dimensions across all embeddings
- Vector_size must match the expected dimension for the model used
- Model_used must be a valid Cohere embedding model

## Metadata
Additional information stored with embeddings including source URL, document hierarchy, timestamps, and processing information.

**Fields**:
- `source_url`: Original URL of the content
- `document_title`: Title of the source document
- `section_hierarchy`: Document hierarchy/outline information
- `chunk_index`: Position within the original document
- `processing_date`: Date when content was processed
- `pipeline_version`: Version of the ingestion pipeline used

**Validation rules**:
- Source_url must be a valid URL
- Processing_date must be a valid timestamp
- Pipeline_version must follow semantic versioning