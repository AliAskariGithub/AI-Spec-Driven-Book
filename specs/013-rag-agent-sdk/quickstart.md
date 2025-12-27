# Quickstart: RAG-Enabled Agent with OpenAI SDK

## Prerequisites

- Python 3.13+
- OpenAI API access
- Cohere API access
- Qdrant Cloud account with populated knowledge base
- Environment variables configured (.env file)

## Setup

1. **Install dependencies**:
   ```bash
   cd fullstack/backend
   pip install openai cohere qdrant-client python-dotenv
   ```

2. **Configure environment**:
   ```bash
   # In your .env file:
   OPENAI_API_KEY=your_openai_api_key
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_cluster_url
   QDRANT_API_KEY=your_qdrant_api_key
   COLLECTION_NAME=your_collection_name
   ```

3. **Verify Qdrant connection**:
   Ensure your Qdrant collection contains the book content that has been ingested via the main.py pipeline.

## Basic Usage

1. **Initialize the agent**:
   ```python
   from agent import RAGAgent

   agent = RAGAgent()
   ```

2. **Query the agent**:
   ```python
   response = agent.query("What is the main concept in Chapter 1?")
   print(response.content)
   print(response.sources)
   ```

3. **Query with specific scope**:
   ```python
   # Full book query
   response = agent.query("Explain the core principles", scope="full-book")

   # Section-specific query
   response = agent.query("What is the name of Module 1?", scope="section", section_id="introduction")
   ```

4. **Agent usage examples**:
   ```python
   from agent import RAGAgent

   # Initialize the agent
   agent = RAGAgent()

   # Basic query
   response = agent.query("What are the key concepts in the book?")
   print(f"Response: {response.content}")
   print(f"Sources: {[source.url for source in response.sources]}")
   print(f"Confidence: {response.confidence}")

   # Scoped query for specific section
   response = agent.query("What does the methodology chapter discuss?", scope="section-specific", section_id="methodology")
   print(f"Response: {response.content}")
   print(f"Sources: {[source.url for source in response.sources]}")

   # Page-specific query
   response = agent.query("What are the conclusions?", scope="page-specific", section_id="conclusion")
   print(f"Response: {response.content}")
   print(f"Sources: {[source.url for source in response.sources]}")
   ```

## Expected Output

The agent will return responses that:
- Are grounded in retrieved content only
- Include source citations with URLs and sections
- Refuse to answer if no relevant content is found
- Demonstrate deterministic behavior for identical queries

## Troubleshooting

- **No results returned**: Verify Qdrant collection has ingested content
- **API errors**: Check that all API keys are correctly configured
- **Poor quality responses**: Verify the ingestion pipeline has properly indexed the book content