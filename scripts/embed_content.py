"""
Embedding Pipeline for Physical AI Robotics Book
Chunks markdown files, generates embeddings via Gemini, uploads to Qdrant
"""

import os
import re
import json
from pathlib import Path
from typing import Generator
from dotenv import load_dotenv
import frontmatter
from google import genai
from google.genai import types
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import numpy as np

# Load environment variables
load_dotenv()

# Configuration
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION", "robotics-book")
EMBEDDING_MODEL = os.getenv("EMBEDDING_MODEL", "gemini-embedding-001")
EMBEDDING_DIMENSIONS = int(os.getenv("EMBEDDING_DIMENSIONS", "768"))

# Paths
DOCS_PATH = Path(__file__).parent.parent / "docs"

# Chunk settings
CHUNK_SIZE = 1500  # characters
CHUNK_OVERLAP = 200  # characters


def init_clients():
    """Initialize Gemini and Qdrant clients"""
    # Gemini
    gemini_client = genai.Client(api_key=GEMINI_API_KEY)
    
    # Qdrant
    qdrant_client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
    )
    
    return gemini_client, qdrant_client


def create_collection(qdrant: QdrantClient):
    """Create Qdrant collection if it doesn't exist"""
    collections = qdrant.get_collections().collections
    collection_names = [c.name for c in collections]
    
    if COLLECTION_NAME not in collection_names:
        qdrant.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(
                size=EMBEDDING_DIMENSIONS,
                distance=Distance.COSINE,
            ),
        )
        print(f"✓ Created collection: {COLLECTION_NAME}")
    else:
        print(f"✓ Collection exists: {COLLECTION_NAME}")


def get_markdown_files() -> list[Path]:
    """Get all markdown files from docs folder"""
    files = list(DOCS_PATH.rglob("*.md"))
    # Filter out index.md and _category_.json
    files = [f for f in files if f.name != "_category_.json"]
    return sorted(files)


def clean_markdown(text: str) -> str:
    """Remove markdown formatting for cleaner embeddings"""
    # Remove code blocks
    text = re.sub(r'```[\s\S]*?```', '[CODE BLOCK]', text)
    # Remove inline code
    text = re.sub(r'`[^`]+`', '', text)
    # Remove links but keep text
    text = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', text)
    # Remove images
    text = re.sub(r'!\[([^\]]*)\]\([^)]+\)', '', text)
    # Remove HTML tags
    text = re.sub(r'<[^>]+>', '', text)
    # Remove headers formatting but keep text
    text = re.sub(r'^#{1,6}\s+', '', text, flags=re.MULTILINE)
    # Remove bold/italic
    text = re.sub(r'\*\*([^*]+)\*\*', r'\1', text)
    text = re.sub(r'\*([^*]+)\*', r'\1', text)
    # Remove extra whitespace
    text = re.sub(r'\n{3,}', '\n\n', text)
    return text.strip()


def chunk_text(text: str, chunk_size: int = CHUNK_SIZE, overlap: int = CHUNK_OVERLAP) -> Generator[str, None, None]:
    """Split text into overlapping chunks"""
    if len(text) <= chunk_size:
        yield text
        return
    
    # Split by paragraphs first
    paragraphs = text.split('\n\n')
    current_chunk = ""
    
    for para in paragraphs:
        if len(current_chunk) + len(para) + 2 <= chunk_size:
            current_chunk += para + "\n\n"
        else:
            if current_chunk:
                yield current_chunk.strip()
            # Start new chunk with overlap
            if overlap > 0 and len(current_chunk) > overlap:
                current_chunk = current_chunk[-overlap:] + para + "\n\n"
            else:
                current_chunk = para + "\n\n"
    
    if current_chunk.strip():
        yield current_chunk.strip()


def process_markdown_file(file_path: Path) -> list[dict]:
    """Process a markdown file into chunks with metadata"""
    with open(file_path, 'r', encoding='utf-8') as f:
        post = frontmatter.load(f)
    
    content = post.content
    metadata = post.metadata
    
    # Get relative path for reference
    relative_path = file_path.relative_to(DOCS_PATH)
    
    # Extract title from frontmatter or first heading
    title = metadata.get('title', file_path.stem)
    module = metadata.get('module', 'general')
    description = metadata.get('description', '')
    
    # Clean and chunk the content
    cleaned_content = clean_markdown(content)
    chunks = list(chunk_text(cleaned_content))
    
    documents = []
    for i, chunk in enumerate(chunks):
        doc = {
            'text': chunk,
            'metadata': {
                'file': str(relative_path),
                'title': title,
                'module': module,
                'description': description,
                'chunk_index': i,
                'total_chunks': len(chunks),
            }
        }
        documents.append(doc)
    
    return documents


def generate_embeddings(gemini: genai.Client, texts: list[str]) -> list[list[float]]:
    """Generate embeddings for a batch of texts using Gemini"""
    result = gemini.models.embed_content(
        model=EMBEDDING_MODEL,
        contents=texts,
        config=types.EmbedContentConfig(
            task_type="RETRIEVAL_DOCUMENT",
            output_dimensionality=EMBEDDING_DIMENSIONS,
        )
    )
    
    embeddings = []
    for embedding in result.embeddings:
        # Normalize the embedding
        vec = np.array(embedding.values)
        normalized = vec / np.linalg.norm(vec)
        embeddings.append(normalized.tolist())
    
    return embeddings


def upload_to_qdrant(qdrant: QdrantClient, documents: list[dict], embeddings: list[list[float]], start_id: int = 0):
    """Upload documents and embeddings to Qdrant"""
    points = []
    for i, (doc, embedding) in enumerate(zip(documents, embeddings)):
        point = PointStruct(
            id=start_id + i,
            vector=embedding,
            payload={
                'text': doc['text'],
                **doc['metadata']
            }
        )
        points.append(point)
    
    qdrant.upsert(
        collection_name=COLLECTION_NAME,
        points=points,
    )
    
    return len(points)


def main():
    print("=" * 50)
    print("Physical AI Robotics Book - Embedding Pipeline")
    print("=" * 50)
    
    # Validate environment
    if not GEMINI_API_KEY:
        print("❌ Error: GEMINI_API_KEY not set")
        return
    if not QDRANT_URL:
        print("❌ Error: QDRANT_URL not set")
        return
    if not QDRANT_API_KEY:
        print("❌ Error: QDRANT_API_KEY not set")
        return
    
    print(f"\n📚 Docs path: {DOCS_PATH}")
    print(f"🔗 Qdrant URL: {QDRANT_URL}")
    print(f"📦 Collection: {COLLECTION_NAME}")
    print(f"📐 Dimensions: {EMBEDDING_DIMENSIONS}")
    
    # Initialize clients
    print("\n🔌 Initializing clients...")
    gemini, qdrant = init_clients()
    print("✓ Gemini client ready")
    print("✓ Qdrant client ready")
    
    # Create collection
    print("\n📦 Setting up collection...")
    create_collection(qdrant)
    
    # Get markdown files
    print("\n📄 Finding markdown files...")
    files = get_markdown_files()
    print(f"✓ Found {len(files)} files")
    
    # Process all files
    all_documents = []
    for file in files:
        docs = process_markdown_file(file)
        all_documents.extend(docs)
        print(f"  📄 {file.name}: {len(docs)} chunks")
    
    print(f"\n📊 Total chunks: {len(all_documents)}")
    
    # Generate embeddings in batches
    print("\n🧠 Generating embeddings...")
    batch_size = 20
    all_embeddings = []
    
    for i in range(0, len(all_documents), batch_size):
        batch = all_documents[i:i + batch_size]
        texts = [doc['text'] for doc in batch]
        embeddings = generate_embeddings(gemini, texts)
        all_embeddings.extend(embeddings)
        print(f"  ✓ Batch {i // batch_size + 1}/{(len(all_documents) + batch_size - 1) // batch_size}")
    
    # Upload to Qdrant
    print("\n📤 Uploading to Qdrant...")
    uploaded = upload_to_qdrant(qdrant, all_documents, all_embeddings)
    print(f"✓ Uploaded {uploaded} vectors")
    
    print("\n" + "=" * 50)
    print("✅ Embedding pipeline complete!")
    print("=" * 50)


if __name__ == "__main__":
    main()
