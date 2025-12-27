"""
Docusaurus RAG Ingestion Pipeline

A complete end-to-end pipeline for extracting content from Docusaurus-based book websites,
chunking the content, generating embeddings, and storing them in Qdrant Cloud.
"""

import os
import argparse
import requests
from bs4 import BeautifulSoup
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http.models import PointStruct, VectorParams, Distance
from dotenv import load_dotenv
import tiktoken
from typing import List, Dict, Any
import time
import logging
import xml.etree.ElementTree as ET

# Load environment variables
load_dotenv()

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class DocusaurusRAGPipeline:
    def __init__(self):
        self.cohere_api_key = os.getenv("COHERE_API_KEY", "COHERE_API_KEY_BACKUP")
        self.qdrant_url = os.getenv("QDRANT_URL")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = os.getenv("COLLECTION_NAME", "testing")

        # Initialize clients
        self.cohere_client = cohere.Client(self.cohere_api_key)
        self.qdrant_client = QdrantClient(url=self.qdrant_url, api_key=self.qdrant_api_key)

        # Initialize tokenizer for chunking
        self.tokenizer = tiktoken.encoding_for_model("gpt-3.5-turbo")

        # Default settings
        self.chunk_size_tokens = int(os.getenv("CHUNK_SIZE_TOKENS", "512"))
        self.chunk_overlap_tokens = int(os.getenv("CHUNK_OVERLAP_TOKENS", "64"))
        self.cohere_model = os.getenv("COHERE_MODEL", "embed-english-v3.0")

    def __enter__(self):
        """Context manager entry"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - cleanup resources"""
        self.cleanup()

    def cleanup(self):
        """Clean up resources like closing client connections"""
        logger.info("Cleaning up pipeline resources...")
        # In this case, Cohere and Qdrant clients don't have explicit close methods
        # but we can set them to None to help with garbage collection
        self.cohere_client = None
        self.qdrant_client = None
        logger.info("Pipeline resources cleaned up")

    def fetch_urls_from_docusaurus(self, base_url: str) -> List[str]:
        """
        Fetch all page URLs from a Docusaurus site or sitemap.xml.

        Args:
            base_url (str): The base URL of the Docusaurus site to crawl or sitemap.xml URL

        Returns:
            List[str]: List of URLs found on the site that match Docusaurus patterns
        """
        logger.info(f"Fetching URLs from {base_url}")

        # Check if the URL is a sitemap.xml file
        if base_url.endswith('sitemap.xml'):
            logger.info("Detected sitemap.xml URL, parsing XML...")
            return self._parse_sitemap_xml(base_url)
        else:
            logger.info("Processing as regular Docusaurus site...")
            return self._crawl_docusaurus_site(base_url)

    def _parse_sitemap_xml(self, sitemap_url: str) -> List[str]:
        """
        Parse a sitemap.xml file and extract URLs.

        Args:
            sitemap_url (str): URL to the sitemap.xml file

        Returns:
            List[str]: List of URLs extracted from the sitemap
        """
        logger.info(f"Parsing sitemap from {sitemap_url}")

        try:
            response = requests.get(sitemap_url)
            response.raise_for_status()

            # Parse the XML content
            root = ET.fromstring(response.content)

            # Extract URLs from the sitemap
            urls = []
            namespace = {'sitemap': 'http://www.sitemaps.org/schemas/sitemap/0.9'}

            # Handle default namespace (no prefix)
            for url_elem in root.findall('.//{http://www.sitemaps.org/schemas/sitemap/0.9}url'):
                loc_elem = url_elem.find('{http://www.sitemaps.org/schemas/sitemap/0.9}loc')
                if loc_elem is not None:
                    url = loc_elem.text.strip()
                    # Only include URLs that are part of the same domain
                    base_domain = sitemap_url.rsplit('/', 2)[0]  # Get base domain from sitemap URL
                    if url.startswith(base_domain):
                        urls.append(url)

            logger.info(f"Found {len(urls)} URLs from sitemap")
            return urls

        except ET.ParseError as e:
            logger.error(f"Error parsing sitemap XML: {str(e)}")
            raise
        except Exception as e:
            logger.error(f"Error fetching or processing sitemap: {str(e)}")
            raise

    def _crawl_docusaurus_site(self, base_url: str) -> List[str]:
        """
        Fetch all page URLs from a Docusaurus site by crawling.

        Args:
            base_url (str): The base URL of the Docusaurus site to crawl

        Returns:
            List[str]: List of URLs found on the site that match Docusaurus patterns
        """
        logger.info(f"Crawling Docusaurus site: {base_url}")

        # Get the main page to find links
        response = requests.get(base_url)
        soup = BeautifulSoup(response.content, 'html.parser')

        urls = set()
        urls.add(base_url)  # Add the base URL

        # Find all links in the navigation and content
        for link in soup.find_all('a', href=True):
            href = link['href']

            # Convert relative URLs to absolute
            if href.startswith('/'):
                full_url = base_url.rstrip('/') + href
            elif href.startswith(base_url):
                full_url = href
            else:
                continue

            # Only include URLs from the same domain
            if full_url.startswith(base_url):
                urls.add(full_url)

        # Limit to common Docusaurus patterns
        filtered_urls = []
        for url in urls:
            if any(pattern in url for pattern in ['.html', '/docs/', '/blog/', '/category/']):
                filtered_urls.append(url)

        logger.info(f"Found {len(filtered_urls)} URLs by crawling")
        return filtered_urls

    def extract_content_from_url(self, url: str) -> Dict[str, Any]:
        """
        Extract clean text content from a single URL.

        Args:
            url (str): The URL to extract content from

        Returns:
            Dict[str, Any]: Dictionary containing the extracted content and metadata,
                           or None if extraction fails
        """
        logger.info(f"Extracting content from {url}")

        try:
            response = requests.get(url)
            response.raise_for_status()

            soup = BeautifulSoup(response.content, 'html.parser')

            # Remove script and style elements
            for script in soup(["script", "style"]):
                script.decompose()

            # Try to find main content area (common Docusaurus selectors)
            main_content = (
                soup.find('main') or
                soup.find('article') or
                soup.find('div', class_='container') or
                soup.find('div', {'role': 'main'}) or
                soup.find('div', class_=lambda x: x and 'doc' in x.lower()) or
                soup
            )

            # Extract text content
            text_content = main_content.get_text(separator=' ')

            # Clean up text
            lines = (line.strip() for line in text_content.splitlines())
            chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
            text_content = ' '.join(chunk for chunk in chunks if chunk)

            # Extract title
            title = ""
            if soup.title:
                title = soup.title.string.strip()
            else:
                title_tag = soup.find('h1')
                if title_tag:
                    title = title_tag.get_text().strip()

            return {
                'url': url,
                'title': title,
                'content': text_content,
                'timestamp': time.time()
            }
        except Exception as e:
            logger.error(f"Error extracting content from {url}: {str(e)}")
            return None

    def chunk_text(self, text: str, max_tokens: int, overlap_tokens: int) -> List[Dict[str, Any]]:
        """
        Split text into chunks based on token count with overlap.

        Args:
            text (str): The text to chunk
            max_tokens (int): Maximum number of tokens per chunk
            overlap_tokens (int): Number of overlapping tokens between chunks

        Returns:
            List[Dict[str, Any]]: List of chunk dictionaries with content and metadata
        """
        logger.info(f"Chunking text of {len(text)} characters")

        # Split text into sentences as a starting point
        sentences = text.split('. ')

        chunks = []
        current_chunk = ""
        current_tokens = 0

        for sentence in sentences:
            # Add a period back to the sentence
            sentence_with_period = sentence + '. '

            # Count tokens in the sentence
            sentence_tokens = len(self.tokenizer.encode(sentence_with_period))

            # If adding this sentence would exceed the limit
            if current_tokens + sentence_tokens > max_tokens:
                # If we have content in the current chunk, save it
                if current_chunk.strip():
                    chunks.append({
                        'content': current_chunk.strip(),
                        'token_count': current_tokens
                    })

                # Start a new chunk with potential overlap
                if sentence_tokens > max_tokens:
                    # If the sentence itself is too long, we need to split it
                    sub_chunks = self._split_long_sentence(sentence_with_period, max_tokens)
                    for sub_chunk in sub_chunks[:-1]:
                        chunks.append(sub_chunk)

                    # Start the next chunk with the last sub-chunk
                    last_sub = sub_chunks[-1]
                    current_chunk = last_sub['content']
                    current_tokens = last_sub['token_count']
                else:
                    # Start new chunk with the current sentence
                    current_chunk = sentence_with_period
                    current_tokens = sentence_tokens
            else:
                # Add sentence to current chunk
                current_chunk += sentence_with_period
                current_tokens += sentence_tokens

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunks.append({
                'content': current_chunk.strip(),
                'token_count': current_tokens
            })

        logger.info(f"Created {len(chunks)} chunks")
        return chunks

    def _split_long_sentence(self, sentence: str, max_tokens: int) -> List[Dict[str, Any]]:
        """
        Split a sentence that's too long into smaller chunks.

        Args:
            sentence (str): The sentence to split
            max_tokens (int): Maximum number of tokens per chunk

        Returns:
            List[Dict[str, Any]]: List of chunk dictionaries with content and metadata
        """
        words = sentence.split(' ')
        chunks = []
        current_chunk = ""
        current_tokens = 0

        for word in words:
            word_with_space = word + ' '
            word_tokens = len(self.tokenizer.encode(word_with_space))

            if current_tokens + word_tokens > max_tokens:
                if current_chunk.strip():
                    chunks.append({
                        'content': current_chunk.strip(),
                        'token_count': current_tokens
                    })

                current_chunk = word_with_space
                current_tokens = word_tokens
            else:
                current_chunk += word_with_space
                current_tokens += word_tokens

        if current_chunk.strip():
            chunks.append({
                'content': current_chunk.strip(),
                'token_count': current_tokens
            })

        return chunks

    def generate_embeddings(self, chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Generate embeddings for text chunks using Cohere with rate limiting and retry logic.

        Args:
            chunks (List[Dict[str, Any]]): List of text chunks to generate embeddings for

        Returns:
            List[Dict[str, Any]]: List of embedding dictionaries with content, embeddings, and metadata
        """
        logger.info(f"Generating embeddings for {len(chunks)} chunks")

        # Prepare texts for embedding (we'll use a simple approach for now)
        texts = [chunk['content'] for chunk in chunks]

        # Handle rate limiting and API errors with retry logic
        max_retries = 3
        retry_delay = 1  # seconds
        embeddings = []

        for i, chunk in enumerate(chunks):
            retry_count = 0
            while retry_count < max_retries:
                try:
                    # Cohere embedding generation (batch in smaller chunks to avoid limits)
                    text_batch = [chunk['content']]
                    response = self.cohere_client.embed(
                        texts=text_batch,
                        model=self.cohere_model,
                        input_type="search_document"  # Specify input type for Cohere API
                    )

                    embeddings.append({
                        'chunk_id': f"chunk_{i}",
                        'content': chunk['content'],
                        'embedding': response.embeddings[0],
                        'metadata': {
                            'chunk_index': i,
                            'token_count': chunk['token_count'],
                            'source_type': 'docusaurus_content',
                            'source_url': chunk.get('source_url', ''),
                            'source_title': chunk.get('source_title', '')
                        }
                    })
                    break  # Success, exit retry loop
                except Exception as e:
                    retry_count += 1
                    logger.warning(f"Attempt {retry_count} failed for chunk {i}: {str(e)}")
                    if retry_count < max_retries:
                        time.sleep(retry_delay * (2 ** retry_count))  # Exponential backoff
                    else:
                        logger.error(f"Failed to generate embedding for chunk {i} after {max_retries} attempts")
                        # Add a null embedding or skip - for now we'll add a placeholder
                        embeddings.append({
                            'chunk_id': f"chunk_{i}_error",
                            'content': chunk['content'],
                            'embedding': [0.0] * 1024,  # Placeholder embedding
                            'metadata': {
                                'chunk_index': i,
                                'token_count': chunk['token_count'],
                                'source_type': 'docusaurus_content',
                                'source_url': chunk.get('source_url', ''),
                                'source_title': chunk.get('source_title', ''),
                                'error': str(e)
                            }
                        })

        logger.info(f"Generated {len(embeddings)} embeddings")
        return embeddings

    def store_embeddings(self, embeddings: List[Dict[str, Any]], collection_name: str):
        """
        Store embeddings in Qdrant Cloud with duplicate prevention.

        Args:
            embeddings (List[Dict[str, Any]]): List of embeddings to store
            collection_name (str): Name of the Qdrant collection to store embeddings in

        Returns:
            None
        """
        logger.info(f"Storing {len(embeddings)} embeddings in Qdrant collection: {collection_name}")

        # Create collection if it doesn't exist
        try:
            self.qdrant_client.get_collection(collection_name)
            logger.info(f"Collection {collection_name} already exists")
        except:
            # Get embedding dimension from first embedding
            embedding_dim = len(embeddings[0]['embedding']) if embeddings else 384

            self.qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(size=embedding_dim, distance=Distance.COSINE)
            )
            logger.info(f"Created new collection {collection_name} with dimension {embedding_dim}")

        # Prepare points for insertion with duplicate prevention
        points = []
        seen_content = set()  # Track content to prevent duplicates

        for i, emb in enumerate(embeddings):
            # Create a hash of the content to identify duplicates
            content_hash = hash(emb['content'])

            if content_hash not in seen_content:
                seen_content.add(content_hash)

                points.append(PointStruct(
                    id=i,  # Use sequential ID
                    vector=emb['embedding'],
                    payload={
                        'content': emb['content'],
                        'chunk_id': emb['chunk_id'],
                        **emb['metadata']
                    }
                ))
            else:
                logger.info(f"Duplicate content detected and skipped for chunk {emb['chunk_id']}")

        # Upload points to Qdrant
        self.qdrant_client.upsert(
            collection_name=collection_name,
            points=points
        )

        logger.info(f"Successfully stored {len(points)} vectors in Qdrant (duplicates removed: {len(embeddings) - len(points)})")

    def run_pipeline(self, url: str):
        """
        Run the complete ingestion pipeline with data validation and performance monitoring.

        Args:
            url (str): The base URL of the Docusaurus site to process

        Returns:
            None
        """
        start_time = time.time()
        logger.info(f"Starting RAG ingestion pipeline for {url}")

        # Step 1: Fetch URLs from the Docusaurus site
        urls_start = time.time()
        urls = self.fetch_urls_from_docusaurus(url)
        urls_time = time.time() - urls_start
        logger.info(f"Found {len(urls)} URLs to process (Time: {urls_time:.2f}s)")

        all_chunks = []

        # Step 2: Extract content from each URL
        content_extraction_start = time.time()
        for i, page_url in enumerate(urls):
            logger.info(f"Processing URL {i+1}/{len(urls)}: {page_url}")

            content_data = self.extract_content_from_url(page_url)
            if content_data:
                # Validate extracted content
                if not content_data.get('content') or len(content_data['content'].strip()) == 0:
                    logger.warning(f"No content extracted from {page_url}, skipping...")
                    continue

                # Step 3: Chunk the content
                chunks = self.chunk_text(
                    content_data['content'],
                    self.chunk_size_tokens,
                    self.chunk_overlap_tokens
                )

                # Validate chunks
                if not chunks or len(chunks) == 0:
                    logger.warning(f"No chunks created from {page_url}, skipping...")
                    continue

                # Add source metadata to chunks
                for chunk in chunks:
                    chunk['source_url'] = content_data['url']
                    chunk['source_title'] = content_data['title']

                all_chunks.extend(chunks)

        content_extraction_time = time.time() - content_extraction_start
        logger.info(f"Content extraction completed: {len(all_chunks)} chunks created (Time: {content_extraction_time:.2f}s)")

        # Step 4: Generate embeddings
        if all_chunks:
            logger.info(f"Processing {len(all_chunks)} chunks for embedding generation")

            # Validate chunks before embedding
            valid_chunks = []
            for chunk in all_chunks:
                if chunk.get('content') and len(chunk['content'].strip()) > 0:
                    valid_chunks.append(chunk)
                else:
                    logger.warning("Found chunk with empty content, skipping...")

            if valid_chunks:
                embeddings_start = time.time()
                embeddings = self.generate_embeddings(valid_chunks)
                embeddings_time = time.time() - embeddings_start

                logger.info(f"Embedding generation completed (Time: {embeddings_time:.2f}s)")

                # Validate embeddings
                if embeddings and len(embeddings) > 0:
                    # Step 5: Store embeddings in Qdrant
                    storage_start = time.time()
                    self.store_embeddings(embeddings, self.collection_name)
                    storage_time = time.time() - storage_start

                    total_time = time.time() - start_time
                    logger.info(f"Pipeline completed successfully!")
                    logger.info(f"  - URLs processed: {len(urls)}")
                    logger.info(f"  - Chunks processed: {len(valid_chunks)}")
                    logger.info(f"  - Embeddings stored: {len(embeddings)}")
                    logger.info(f"  - Total time: {total_time:.2f}s")
                    logger.info(f"  - URL fetching: {urls_time:.2f}s")
                    logger.info(f"  - Content extraction: {content_extraction_time:.2f}s")
                    logger.info(f"  - Embedding generation: {embeddings_time:.2f}s")
                    logger.info(f"  - Storage: {storage_time:.2f}s")
                else:
                    logger.warning("No valid embeddings were generated, pipeline stopped.")
            else:
                logger.warning("No valid chunks to process, pipeline stopped.")
        else:
            logger.warning("No content was extracted from the provided URLs.")
            total_time = time.time() - start_time
            logger.info(f"Pipeline completed in {total_time:.2f}s but no content was processed.")


def validate_url(url: str) -> bool:
    """Validate the input URL"""
    import re
    # Basic URL validation regex
    url_pattern = re.compile(
        r'^https?://'  # http:// or https://
        r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # domain...
        r'localhost|'  # localhost...
        r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # ...or ip
        r'(?::\d+)?'  # optional port
        r'(?:/?|[/?]\S+)$', re.IGNORECASE)
    return url_pattern.match(url) is not None


def main():
    parser = argparse.ArgumentParser(description="Docusaurus RAG Ingestion Pipeline")
    parser.add_argument("--url", type=str, required=True, help="URL of the Docusaurus site to process")
    parser.add_argument("--chunk-size", type=int, default=512, help="Maximum tokens per chunk (default: 512)")
    parser.add_argument("--overlap", type=int, default=64, help="Token overlap between chunks (default: 64)")
    parser.add_argument("--collection", type=str, default=("ragchatbot-embedding", "testing"), help="Qdrant collection name (default: ragchatbot-embedding)")

    args = parser.parse_args()

    # Validate URL
    if not validate_url(args.url):
        logger.error(f"Invalid URL provided: {args.url}")
        return 1

    # Validate chunk size
    if args.chunk_size <= 0:
        logger.error(f"Chunk size must be positive, got: {args.chunk_size}")
        return 1

    # Validate overlap
    if args.overlap < 0:
        logger.error(f"Overlap must be non-negative, got: {args.overlap}")
        return 1

    # Validate collection name
    if not args.collection or len(args.collection.strip()) == 0:
        logger.error("Collection name cannot be empty")
        return 1

    # Update environment variables with command line args if provided
    os.environ["CHUNK_SIZE_TOKENS"] = str(args.chunk_size)
    os.environ["CHUNK_OVERLAP_TOKENS"] = str(args.overlap)
    os.environ["COLLECTION_NAME"] = args.collection

    try:
        # Create and run the pipeline with proper resource management
        pipeline = DocusaurusRAGPipeline()
        pipeline.run_pipeline(args.url)
        pipeline.cleanup()  # Manual cleanup
        return 0
    except Exception as e:
        logger.error(f"Pipeline execution failed: {str(e)}")
        return 1


if __name__ == "__main__":
    main()