# 📊 RAG Chunking Strategy

> This document defines the chunking strategy for embedding the Physical AI & Humanoid Robotics textbook into a vector database for RAG-based retrieval.

---

## 🎯 Overview

The goal is to create chunks that:
1. **Stand alone** - Each chunk makes sense without surrounding context
2. **Are retrievable** - Match user queries semantically
3. **Maintain context** - Include enough information for accurate responses
4. **Preserve code** - Keep code blocks intact for executable examples

---

## 📐 Chunk Size Configuration

### Recommended Settings

```python
# Primary chunking configuration
CHUNK_CONFIG = {
    "min_chunk_size": 300,    # tokens - minimum viable context
    "max_chunk_size": 500,    # tokens - optimal for embedding models
    "target_chunk_size": 400, # tokens - sweet spot
    "overlap_size": 50,       # tokens - context preservation
    "overlap_percentage": 0.1 # 10% overlap between chunks
}
```

### Why These Numbers?

| Parameter | Value | Reasoning |
|-----------|-------|-----------|
| Min size | 300 tokens | Ensures enough context for meaningful retrieval |
| Max size | 500 tokens | Stays within embedding model sweet spot |
| Target | 400 tokens | Balances context with specificity |
| Overlap | 50 tokens | Preserves cross-chunk context |

### Model-Specific Considerations

```python
# OpenAI text-embedding-3-small
MAX_INPUT_TOKENS = 8191
OPTIMAL_RANGE = (256, 512)

# OpenAI text-embedding-3-large
MAX_INPUT_TOKENS = 8191
OPTIMAL_RANGE = (256, 512)

# Cohere embed-english-v3.0
MAX_INPUT_TOKENS = 512
OPTIMAL_RANGE = (256, 512)
```

---

## ✂️ Splitting Strategy

### Primary Split Points

Split content at these markers (in order of priority):

```python
SPLIT_HIERARCHY = [
    "## ",      # H2 headers - major sections
    "### ",     # H3 headers - subsections  
    "\n\n",     # Double newline - paragraphs
    ". ",       # Sentence boundaries (fallback)
]
```

### Content-Aware Rules

```python
SPLITTING_RULES = {
    # Code blocks should never be split
    "code_blocks": {
        "split": False,
        "include_surrounding_context": True,
        "context_lines": 2  # Include 2 lines before/after
    },
    
    # TL;DR sections are high-priority chunks
    "tldr_sections": {
        "split": False,
        "boost_priority": 1.5  # Higher retrieval weight
    },
    
    # Tables should stay intact
    "tables": {
        "split": False,
        "include_header": True
    },
    
    # Lists can be split if needed
    "lists": {
        "split_at_items": True,
        "min_items_per_chunk": 3
    }
}
```

---

## 🏗️ Content Hierarchy

### Document Structure

```
Document
├── Frontmatter (metadata - not embedded, used for filtering)
├── Title (H1)
├── TL;DR Block (priority chunk)
├── Section (H2)
│   ├── Introduction paragraph
│   ├── Subsection (H3)
│   │   ├── Explanation
│   │   └── Code example
│   └── Subsection (H3)
├── Section (H2)
│   └── ...
└── Summary/Next Steps
```

### Chunk Priority Levels

```python
CHUNK_PRIORITIES = {
    "tldr": 1.0,           # Always retrieve
    "introduction": 0.9,   # High priority
    "concept_explanation": 0.8,
    "code_example": 0.7,
    "troubleshooting": 0.9,
    "summary": 0.8,
    "reference": 0.5       # Lower priority
}
```

---

## 📦 Metadata Enrichment

Each chunk should include:

```python
CHUNK_METADATA = {
    # Document-level
    "doc_id": "ros2-nodes-topics-services",
    "doc_title": "Nodes, Topics, and Services",
    "module": 1,
    "module_title": "The Robotic Nervous System (ROS 2)",
    
    # Section-level
    "section_id": "topics",
    "section_title": "Topics: Broadcast Communication",
    "section_type": "concept",  # concept, tutorial, reference, troubleshooting
    
    # Chunk-level
    "chunk_index": 3,
    "total_chunks": 12,
    "has_code": True,
    "code_language": "python",
    
    # Semantic
    "keywords": ["topics", "publishers", "subscribers", "pub/sub"],
    "difficulty": "beginner",
    "content_type": "explanation"
}
```

---

## 🔄 Overlap Strategy

### Standard Overlap

```python
def create_overlapping_chunks(text, chunk_size=400, overlap=50):
    """
    Create chunks with overlap for context preservation.
    
    Overlap ensures:
    - Cross-references work across chunk boundaries
    - Concepts that span boundaries are retrievable
    - Context is maintained for coherent responses
    """
    chunks = []
    start = 0
    
    while start < len(text):
        end = start + chunk_size
        
        # Find natural break point
        break_point = find_natural_break(text, end)
        
        chunk = text[start:break_point]
        chunks.append(chunk)
        
        # Move start with overlap
        start = break_point - overlap
    
    return chunks
```

### Contextual Overlap

For code-heavy sections, include more context:

```python
CODE_CONTEXT_CONFIG = {
    "before_code": 100,  # tokens before code block
    "after_code": 50,    # tokens after code block
    "include_comments": True,
    "include_imports": True  # Always include import statements
}
```

---

## 🎛️ Per-Module Configuration

Each module's `metadata.json` contains section-specific chunking hints:

```json
{
  "sections": [
    {
      "id": "ros2-nodes-topics-services",
      "chunkingHints": {
        "primarySections": ["Nodes", "Topics", "Services"],
        "minChunkSize": 300,
        "maxChunkSize": 500,
        "splitOn": ["##", "###"],
        "keepCodeBlocksIntact": true
      }
    }
  ]
}
```

---

## 🔍 Retrieval Optimization

### Query Enhancement

```python
def enhance_query(user_query):
    """
    Enhance user query for better retrieval.
    """
    enhancements = {
        # Expand abbreviations
        "ROS": "ROS Robot Operating System",
        "URDF": "URDF Unified Robot Description Format",
        "VLA": "VLA Vision-Language-Action",
        
        # Add context
        "how to": "tutorial guide steps",
        "what is": "definition explanation concept",
        "error": "troubleshooting problem fix",
    }
    
    enhanced = user_query
    for abbr, expansion in enhancements.items():
        if abbr.lower() in user_query.lower():
            enhanced += f" {expansion}"
    
    return enhanced
```

### Hybrid Search

```python
SEARCH_CONFIG = {
    "vector_weight": 0.7,      # Semantic similarity
    "keyword_weight": 0.3,     # BM25/keyword matching
    "top_k": 5,                # Initial retrieval
    "rerank_top_k": 3,         # After reranking
    "score_threshold": 0.7,    # Minimum relevance
    "mmr_lambda": 0.5          # Diversity vs relevance
}
```

---

## 📊 Embedding Pipeline

### Processing Steps

```python
PIPELINE_STEPS = [
    "1. Load markdown files",
    "2. Parse frontmatter (extract metadata)",
    "3. Identify special blocks (TL;DR, code, tables)",
    "4. Split at H2/H3 boundaries",
    "5. Further split if chunks > max_size",
    "6. Apply overlap",
    "7. Enrich with metadata",
    "8. Generate embeddings",
    "9. Store in vector database"
]
```

### Sample Implementation

```python
from langchain.text_splitter import MarkdownHeaderTextSplitter
from langchain.embeddings import OpenAIEmbeddings
from qdrant_client import QdrantClient

def process_document(file_path):
    """Process a single markdown document."""
    
    # 1. Load and parse
    with open(file_path, 'r') as f:
        content = f.read()
    
    frontmatter, body = parse_frontmatter(content)
    
    # 2. Split by headers
    headers_to_split = [
        ("#", "title"),
        ("##", "section"),
        ("###", "subsection"),
    ]
    
    splitter = MarkdownHeaderTextSplitter(
        headers_to_split_on=headers_to_split
    )
    
    chunks = splitter.split_text(body)
    
    # 3. Enrich chunks
    enriched_chunks = []
    for i, chunk in enumerate(chunks):
        enriched = {
            "content": chunk.page_content,
            "metadata": {
                **frontmatter,
                **chunk.metadata,
                "chunk_index": i,
                "file_path": file_path
            }
        }
        enriched_chunks.append(enriched)
    
    return enriched_chunks

def embed_and_store(chunks, collection_name="robotics-book"):
    """Embed chunks and store in Qdrant."""
    
    embeddings = OpenAIEmbeddings(model="text-embedding-3-small")
    client = QdrantClient(url="http://localhost:6333")
    
    for chunk in chunks:
        vector = embeddings.embed_query(chunk["content"])
        
        client.upsert(
            collection_name=collection_name,
            points=[{
                "id": generate_chunk_id(chunk),
                "vector": vector,
                "payload": chunk["metadata"]
            }]
        )
```

---

## 📁 File Reference

| File | Purpose |
|------|---------|
| `docs/module-*/metadata.json` | Per-module chunking hints |
| `memory.md` | RAG configuration reference |
| This file | Chunking strategy documentation |

---

## 🧪 Testing Chunks

### Quality Checklist

- [ ] Each chunk makes sense in isolation
- [ ] Code blocks are complete and runnable
- [ ] TL;DR sections are intact
- [ ] Metadata is accurate
- [ ] Keywords match content
- [ ] No orphaned fragments

### Test Queries

```python
TEST_QUERIES = [
    "What is a ROS 2 node?",
    "How do I create a publisher in Python?",
    "What's the difference between Gazebo and Unity?",
    "How to simulate LiDAR sensors?",
    "What is Visual SLAM in Isaac ROS?",
    "How do I integrate Whisper for voice commands?",
    "What is cognitive planning with LLMs?",
]
```

---

*Last Updated: December 18, 2025*
