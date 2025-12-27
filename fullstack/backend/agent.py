import os
import json
import logging
import time
from dotenv import load_dotenv
from agents import Agent, Runner, function_tool

# Load environment variables
load_dotenv()

# Configure OpenAI to use OpenRouter
openrouter_api_key = os.getenv("OPENROUTER_API_KEY")
if not openrouter_api_key:
    raise ValueError("OPENROUTER_API_KEY is not set in your .env file.")

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@function_tool
def retrieve_information(query: str):
    """
    Retrieve relevant information from knowledge base using RAGRetrievalValidator.

    Args:
        query: The search query to find relevant information

    Returns:
        A dictionary containing the query, retrieved chunks, and total results
    """
    try:
        from retrieves import RAGRetrievalValidator
        validator = RAGRetrievalValidator()

        json_response = validator.retrieve(query_text=query, top_k=5, threshold=0.0)  # Lower threshold to capture more results
        results = json.loads(json_response)

        # Check if there's an error in the response
        if 'error' in results:
            logger.error(f"Retrieval error: {results['error']}")
            return {
                'query': query,
                'retrieved_chunks': [],
                'total_results': 0,
                'error': results['error']
            }

        formatted_results = [
            {
                'content': r['content'],
                'url': r.get('url', r.get('source_url', '')),  # Handle both possible field names
                'position': r.get('position', r.get('chunk_index', 0)),  # Handle both possible field names
                'similarity_score': r['similarity_score']
            }
            for r in results.get('results', [])
        ]
        return {
            'query': query,
            'retrieved_chunks': formatted_results,
            'total_results': len(formatted_results)
        }
    except ImportError as e:
        logger.error(f"Import error in retrieve_information: {e}")
        return {
            'query': query,
            'retrieved_chunks': [],
            'total_results': 0,
            'error': f"Import error: {str(e)}"
        }
    except json.JSONDecodeError as e:
        logger.error(f"JSON decode error in retrieve_information: {e}")
        return {
            'query': query,
            'retrieved_chunks': [],
            'total_results': 0,
            'error': f"JSON decode error: {str(e)}"
        }
    except Exception as e:
        logger.error(f"Error in retrieve_information: {e}")
        return {
            'query': query,
            'retrieved_chunks': [],
            'total_results': 0,
            'error': str(e)
        }

# Initialize agent with OpenAI Agent SDK - using OpenRouter model via LiteLLM
agent = Agent(
    name="RAG Assistant",
    instructions=(
        "You are a helpful assistant that answers questions based on retrieved documents. "
        "When asked a question, retrieve relevant documents first using the retrieve_information tool, "
        "then answer based on them. Always cite your sources and provide the information that was used "
        "to generate the answer. If the retrieved information doesn't directly answer the question, "
        "explain what information was found and why it may not be sufficient."
    ),
    tools=[retrieve_information],
    model="mistralai/devstral-2512:free"  # Use Mistral: Devstral 2 2512 (free) model via OpenRouter through LiteLLM
)

# Helper to calculate confidence
def calculate_confidence(chunks):
    if not chunks:
        return "low"
    avg_score = sum(c.get("similarity_score", 0) for c in chunks) / len(chunks)
    if avg_score >= 0.7:
        return "high"
    elif avg_score >= 0.4:
        return "medium"
    return "low"

# Synchronous query function
def query_sync(query_text: str):
    """
    Synchronous function to query the agent and return structured response.

    Args:
        query_text: The question to ask the agent

    Returns:
        A dictionary containing the answer, sources, matched chunks, and confidence
    """
    logger.info(f"Querying agent: {query_text}")

    # Run the agent synchronously - using OpenAI Agent SDK
    # We'll pass a custom run_config to potentially handle OpenRouter
    result = Runner.run_sync(agent, input=query_text)
    answer = result.final_output

    # Extract tool results and matched chunks
    tool_results = getattr(result, 'tools_used', [])
    matched_chunks = []
    sources = set()

    for tr in tool_results:
        if hasattr(tr, 'tool_name') and tr.tool_name == "retrieve_information":
            data = getattr(tr, 'output', {})
            retrieved_chunks = data.get("retrieved_chunks", [])
            matched_chunks.extend(retrieved_chunks)
            sources.update(chunk.get("url", "") for chunk in retrieved_chunks if chunk.get("url"))

    response = {
        "answer": answer,
        "sources": list(sources),
        "matched_chunks": matched_chunks,
        "confidence": calculate_confidence(matched_chunks)
    }
    return response

# Main function to run multiple queries
def main():
    """
    Main function to demonstrate the agent functionality.
    """
    logger.info("Initializing RAG Agent with OpenAI Agent SDK...")

    # Test queries to demonstrate functionality
    test_queries = [
        "Describe Module 1?",
        "What is the learning objective of Module 2?"
    ]

    print("RAG Agent with OpenAI Agent SDK - Testing Queries")
    print("=" * 50)

    for i, query in enumerate(test_queries, 1):
        print(f"\nQuery {i}: {query}")
        print("-" * 30)

        response = query_sync(query)

        print(f"Answer: {response['answer']}")

        if response.get("sources"):
            print(f"Sources: {len(response['sources'])} documents")
            for source in response['sources'][:3]:  # Show first 3 sources
                print(f"  - {source}")

        if response.get("matched_chunks"):
            print(f"Matched chunks: {len(response['matched_chunks'])}")
            for j, chunk in enumerate(response['matched_chunks'][:2], 1):  # Show first 2 chunks
                content_preview = chunk['content'][:100] + "..." if len(chunk['content']) > 100 else chunk['content']
                print(f"  Chunk {j}: {content_preview}")
                print(f"    Source: {chunk['url']}")
                print(f"    Score: {chunk['similarity_score']:.3f}")

        print(f"Confidence: {response['confidence']}")
        if i < len(test_queries):
            time.sleep(1)  # small delay between queries

if __name__ == "__main__":
    main()