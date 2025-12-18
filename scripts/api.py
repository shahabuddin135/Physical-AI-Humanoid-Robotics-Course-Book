"""
RAG API for Physical AI Robotics Book
FastAPI backend with streaming support
"""

import os
from typing import AsyncGenerator
from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from google import genai
from google.genai import types
from qdrant_client import QdrantClient
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
LLM_MODEL = os.getenv("LLM_MODEL", "gemini-2.5-flash")

# Initialize FastAPI
app = FastAPI(
    title="ROBO-SAGE API",
    description="RAG-powered chatbot for Physical AI & Humanoid Robotics Book",
    version="1.0.0",
)

# CORS for frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, restrict to your domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize clients
gemini_client = genai.Client(api_key=GEMINI_API_KEY)
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)


# Request/Response models
class ChatRequest(BaseModel):
    message: str
    top_k: int = 5


class ChatResponse(BaseModel):
    response: str
    sources: list[dict]


class HealthResponse(BaseModel):
    status: str
    qdrant: bool
    gemini: bool


# System prompt for the chatbot
SYSTEM_PROMPT = """You are ROBO-SAGE, the AI assistant for the Physical AI & Humanoid Robotics textbook.

Your personality:
- Helpful and knowledgeable about robotics
- Occasionally witty but always professional
- You explain complex concepts in simple terms
- You reference the book content when answering

You have been provided with relevant excerpts from the textbook to answer the user's question.
Always base your answers on the provided context. If the context doesn't contain relevant information,
say so honestly but try to provide general guidance.

Format your responses clearly:
- Use bullet points for lists
- Use code blocks for code examples
- Keep responses concise but thorough
- Cite which section the information comes from when relevant"""


def get_query_embedding(query: str) -> list[float]:
    """Generate embedding for a search query"""
    result = gemini_client.models.embed_content(
        model=EMBEDDING_MODEL,
        contents=query,
        config=types.EmbedContentConfig(
            task_type="RETRIEVAL_QUERY",
            output_dimensionality=EMBEDDING_DIMENSIONS,
        )
    )
    
    vec = np.array(result.embeddings[0].values)
    normalized = vec / np.linalg.norm(vec)
    return normalized.tolist()


def search_documents(query_embedding: list[float], top_k: int = 5) -> list[dict]:
    """Search Qdrant for relevant documents"""
    results = qdrant_client.query_points(
        collection_name=COLLECTION_NAME,
        query=query_embedding,
        limit=top_k,
    ).points
    
    documents = []
    for result in results:
        documents.append({
            "text": result.payload.get("text", ""),
            "title": result.payload.get("title", ""),
            "file": result.payload.get("file", ""),
            "module": result.payload.get("module", ""),
            "score": result.score,
        })
    
    return documents


def build_context(documents: list[dict]) -> str:
    """Build context string from retrieved documents"""
    context_parts = []
    for i, doc in enumerate(documents, 1):
        context_parts.append(
            f"[Source {i}: {doc['title']} ({doc['file']})]\n{doc['text']}"
        )
    return "\n\n---\n\n".join(context_parts)


async def generate_streaming_response(query: str, context: str) -> AsyncGenerator[str, None]:
    """Generate streaming response from Gemini"""
    prompt = f"""Context from the Physical AI & Humanoid Robotics textbook:

{context}

---

User Question: {query}

Please provide a helpful answer based on the context above."""

    response = gemini_client.models.generate_content_stream(
        model=LLM_MODEL,
        contents=prompt,
        config=types.GenerateContentConfig(
            system_instruction=SYSTEM_PROMPT,
            temperature=0.7,
            max_output_tokens=1024,
        )
    )
    
    for chunk in response:
        if chunk.text:
            yield chunk.text


@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Check API health and service connectivity"""
    qdrant_ok = False
    gemini_ok = False
    
    try:
        qdrant_client.get_collections()
        qdrant_ok = True
    except Exception:
        pass
    
    try:
        gemini_client.models.list()
        gemini_ok = True
    except Exception:
        pass
    
    return HealthResponse(
        status="ok" if qdrant_ok and gemini_ok else "degraded",
        qdrant=qdrant_ok,
        gemini=gemini_ok,
    )


@app.post("/chat")
async def chat(request: ChatRequest):
    """Chat endpoint with RAG retrieval"""
    try:
        # 1. Generate query embedding
        query_embedding = get_query_embedding(request.message)
        
        # 2. Search for relevant documents
        documents = search_documents(query_embedding, top_k=request.top_k)
        
        if not documents:
            return ChatResponse(
                response="I couldn't find relevant information in the textbook. Could you rephrase your question?",
                sources=[],
            )
        
        # 3. Build context
        context = build_context(documents)
        
        # 4. Generate response (non-streaming for simple JSON response)
        prompt = f"""Context from the Physical AI & Humanoid Robotics textbook:

{context}

---

User Question: {request.message}

Please provide a helpful answer based on the context above."""

        response = gemini_client.models.generate_content(
            model=LLM_MODEL,
            contents=prompt,
            config=types.GenerateContentConfig(
                system_instruction=SYSTEM_PROMPT,
                temperature=0.7,
                max_output_tokens=1024,
            )
        )
        
        return ChatResponse(
            response=response.text,
            sources=[{
                "title": doc["title"],
                "file": doc["file"],
                "score": doc["score"],
            } for doc in documents[:3]],
        )
    
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/chat/stream")
async def chat_stream(request: ChatRequest):
    """Streaming chat endpoint with RAG retrieval"""
    try:
        # 1. Generate query embedding
        query_embedding = get_query_embedding(request.message)
        
        # 2. Search for relevant documents
        documents = search_documents(query_embedding, top_k=request.top_k)
        
        if not documents:
            async def no_results():
                yield "I couldn't find relevant information in the textbook. Could you rephrase your question?"
            return StreamingResponse(no_results(), media_type="text/plain")
        
        # 3. Build context
        context = build_context(documents)
        
        # 4. Generate streaming response
        return StreamingResponse(
            generate_streaming_response(request.message, context),
            media_type="text/plain",
        )
    
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
