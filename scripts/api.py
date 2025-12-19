"""
RAG API for Physical AI Robotics Book
FastAPI backend with streaming support and authentication
"""

import os
import hashlib
import secrets
from datetime import datetime, timedelta
from typing import AsyncGenerator, Optional
from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException, Depends, Header
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel, EmailStr
from google import genai
from google.genai import types
from qdrant_client import QdrantClient
import numpy as np
import psycopg2
from psycopg2.extras import RealDictCursor

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
DATABASE_URL = os.getenv("DATABASE_URL")

# Initialize FastAPI
app = FastAPI(
    title="Physical AI Robotics API",
    description="RAG-powered chatbot with auth for Physical AI & Humanoid Robotics Book",
    version="2.0.0",
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


# Database helper functions
def get_db_connection():
    """Get a PostgreSQL database connection"""
    return psycopg2.connect(DATABASE_URL, cursor_factory=RealDictCursor)


def init_database():
    """Initialize database tables"""
    conn = get_db_connection()
    cur = conn.cursor()
    
    # Users table
    cur.execute("""
        CREATE TABLE IF NOT EXISTS users (
            id SERIAL PRIMARY KEY,
            email VARCHAR(255) UNIQUE NOT NULL,
            password_hash VARCHAR(255) NOT NULL,
            name VARCHAR(255),
            software_level VARCHAR(50) DEFAULT 'beginner',
            hardware_level VARCHAR(50) DEFAULT 'none',
            programming_languages TEXT,
            robotics_experience VARCHAR(50) DEFAULT 'none',
            chat_messages_used INTEGER DEFAULT 0,
            last_message_reset TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """)
    
    # Sessions table
    cur.execute("""
        CREATE TABLE IF NOT EXISTS sessions (
            id SERIAL PRIMARY KEY,
            user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,
            token VARCHAR(255) UNIQUE NOT NULL,
            expires_at TIMESTAMP NOT NULL,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """)
    
    # User edits table (for personalized book edits)
    cur.execute("""
        CREATE TABLE IF NOT EXISTS user_edits (
            id SERIAL PRIMARY KEY,
            user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,
            page_path VARCHAR(255) NOT NULL,
            original_content TEXT,
            edited_content TEXT NOT NULL,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            UNIQUE(user_id, page_path)
        )
    """)
    
    conn.commit()
    cur.close()
    conn.close()


def hash_password(password: str) -> str:
    """Hash a password using SHA-256 with salt"""
    salt = secrets.token_hex(16)
    hashed = hashlib.sha256((password + salt).encode()).hexdigest()
    return f"{salt}${hashed}"


def verify_password(password: str, password_hash: str) -> bool:
    """Verify a password against its hash"""
    try:
        salt, hashed = password_hash.split("$")
        return hashlib.sha256((password + salt).encode()).hexdigest() == hashed
    except ValueError:
        return False


def generate_session_token() -> str:
    """Generate a secure session token"""
    return secrets.token_urlsafe(32)


async def get_current_user(authorization: Optional[str] = Header(None)):
    """Get current user from session token"""
    if not authorization:
        return None
    
    token = authorization.replace("Bearer ", "")
    
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        
        cur.execute("""
            SELECT u.* FROM users u
            JOIN sessions s ON u.id = s.user_id
            WHERE s.token = %s AND s.expires_at > NOW()
        """, (token,))
        
        user = cur.fetchone()
        cur.close()
        conn.close()
        
        return dict(user) if user else None
    except Exception:
        return None


# Request/Response models
class SignupRequest(BaseModel):
    email: EmailStr
    password: str
    name: str
    software_level: str = "beginner"
    hardware_level: str = "none"
    programming_languages: str = ""
    robotics_experience: str = "none"


class SigninRequest(BaseModel):
    email: EmailStr
    password: str


class AuthResponse(BaseModel):
    token: str
    user: dict


class UserEditRequest(BaseModel):
    page_path: str
    content: str


class PersonalizeRequest(BaseModel):
    content: str
    user_level: str


class TranslateRequest(BaseModel):
    content: str
    target_language: str = "urdu"


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


@app.on_event("startup")
async def startup_event():
    """Initialize database on startup"""
    if DATABASE_URL:
        try:
            init_database()
            print("Database initialized successfully")
        except Exception as e:
            print(f"Database initialization error: {e}")


@app.post("/auth/signup", response_model=AuthResponse)
async def signup(request: SignupRequest):
    """Register a new user"""
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        
        # Check if user exists
        cur.execute("SELECT id FROM users WHERE email = %s", (request.email,))
        if cur.fetchone():
            cur.close()
            conn.close()
            raise HTTPException(status_code=400, detail="Email already registered")
        
        # Create user
        password_hash = hash_password(request.password)
        cur.execute("""
            INSERT INTO users (email, password_hash, name, software_level, hardware_level, 
                             programming_languages, robotics_experience)
            VALUES (%s, %s, %s, %s, %s, %s, %s)
            RETURNING id, email, name, software_level, hardware_level, programming_languages, robotics_experience
        """, (request.email, password_hash, request.name, request.software_level,
              request.hardware_level, request.programming_languages, request.robotics_experience))
        
        user = dict(cur.fetchone())
        
        # Create session
        token = generate_session_token()
        expires_at = datetime.now() + timedelta(days=7)
        cur.execute("""
            INSERT INTO sessions (user_id, token, expires_at)
            VALUES (%s, %s, %s)
        """, (user["id"], token, expires_at))
        
        conn.commit()
        cur.close()
        conn.close()
        
        return AuthResponse(token=token, user=user)
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/auth/signin", response_model=AuthResponse)
async def signin(request: SigninRequest):
    """Sign in an existing user"""
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        
        cur.execute("SELECT * FROM users WHERE email = %s", (request.email,))
        user = cur.fetchone()
        
        if not user or not verify_password(request.password, user["password_hash"]):
            cur.close()
            conn.close()
            raise HTTPException(status_code=401, detail="Invalid email or password")
        
        user = dict(user)
        del user["password_hash"]
        
        # Create new session
        token = generate_session_token()
        expires_at = datetime.now() + timedelta(days=7)
        cur.execute("""
            INSERT INTO sessions (user_id, token, expires_at)
            VALUES (%s, %s, %s)
        """, (user["id"], token, expires_at))
        
        conn.commit()
        cur.close()
        conn.close()
        
        return AuthResponse(token=token, user=user)
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/auth/signout")
async def signout(authorization: Optional[str] = Header(None)):
    """Sign out and invalidate session"""
    if not authorization:
        return {"success": True}
    
    token = authorization.replace("Bearer ", "")
    
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        cur.execute("DELETE FROM sessions WHERE token = %s", (token,))
        conn.commit()
        cur.close()
        conn.close()
    except Exception:
        pass
    
    return {"success": True}


@app.get("/auth/me")
async def get_me(user = Depends(get_current_user)):
    """Get current authenticated user"""
    if not user:
        raise HTTPException(status_code=401, detail="Not authenticated")
    return user


@app.post("/personalize")
async def personalize_content(request: PersonalizeRequest, user = Depends(get_current_user)):
    """Personalize content based on user's level"""
    if not user:
        raise HTTPException(status_code=401, detail="Authentication required")
    
    level_prompts = {
        "beginner": "Explain this content for a complete beginner with no prior robotics knowledge. Use simple analogies and avoid jargon.",
        "intermediate": "Adapt this for someone with basic programming and some robotics knowledge. Include practical tips.",
        "advanced": "Enhance this for an experienced engineer. Add technical depth and advanced implementation details."
    }
    
    prompt = f"""{level_prompts.get(request.user_level, level_prompts["beginner"])}

Content to personalize:
{request.content}

Provide the personalized version in the same Markdown format."""

    try:
        response = gemini_client.models.generate_content(
            model=LLM_MODEL,
            contents=prompt,
            config=types.GenerateContentConfig(
                temperature=0.7,
                max_output_tokens=2048,
            )
        )
        return {"content": response.text}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/translate")
async def translate_content(request: TranslateRequest, user = Depends(get_current_user)):
    """Translate content to Urdu or other language"""
    if not user:
        raise HTTPException(status_code=401, detail="Authentication required")
    
    prompt = f"""Translate the following technical content to {request.target_language}. 
Keep technical terms in English where appropriate (like ROS, URDF, etc.) but translate the explanations.
Maintain the Markdown formatting.

Content:
{request.content}

Translated content:"""

    try:
        response = gemini_client.models.generate_content(
            model=LLM_MODEL,
            contents=prompt,
            config=types.GenerateContentConfig(
                temperature=0.3,
                max_output_tokens=2048,
            )
        )
        return {"content": response.text}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/user/edits")
async def save_user_edit(request: UserEditRequest, user = Depends(get_current_user)):
    """Save a user's personal edit to a page"""
    if not user:
        raise HTTPException(status_code=401, detail="Authentication required")
    
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        
        cur.execute("""
            INSERT INTO user_edits (user_id, page_path, edited_content)
            VALUES (%s, %s, %s)
            ON CONFLICT (user_id, page_path) 
            DO UPDATE SET edited_content = EXCLUDED.edited_content, updated_at = CURRENT_TIMESTAMP
            RETURNING id
        """, (user["id"], request.page_path, request.content))
        
        conn.commit()
        cur.close()
        conn.close()
        
        return {"success": True}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/user/edits/{page_path:path}")
async def get_user_edit(page_path: str, user = Depends(get_current_user)):
    """Get a user's personal edit for a page"""
    if not user:
        return {"content": None}
    
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        
        cur.execute("""
            SELECT edited_content FROM user_edits
            WHERE user_id = %s AND page_path = %s
        """, (user["id"], page_path))
        
        result = cur.fetchone()
        cur.close()
        conn.close()
        
        return {"content": result["edited_content"] if result else None}
    except Exception:
        return {"content": None}


@app.get("/chat/limit")
async def get_chat_limit(user = Depends(get_current_user)):
    """Get user's chat message limit status"""
    if not user:
        # Anonymous user - check localStorage on frontend
        return {"limit": 3, "used": 0, "authenticated": False}
    
    # Check if we need to reset the daily counter
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        
        cur.execute("SELECT chat_messages_used, last_message_reset FROM users WHERE id = %s", (user["id"],))
        result = cur.fetchone()
        
        messages_used = result["chat_messages_used"]
        last_reset = result["last_message_reset"]
        
        # Reset if it's been more than 24 hours
        if datetime.now() - last_reset > timedelta(hours=24):
            cur.execute("""
                UPDATE users SET chat_messages_used = 0, last_message_reset = NOW()
                WHERE id = %s
            """, (user["id"],))
            conn.commit()
            messages_used = 0
        
        cur.close()
        conn.close()
        
        return {"limit": 20, "used": messages_used, "authenticated": True}
    except Exception:
        return {"limit": 20, "used": 0, "authenticated": True}


@app.post("/chat/increment")
async def increment_chat_count(user = Depends(get_current_user)):
    """Increment the user's chat message count"""
    if not user:
        return {"success": True}  # Anonymous handled on frontend
    
    try:
        conn = get_db_connection()
        cur = conn.cursor()
        
        cur.execute("""
            UPDATE users SET chat_messages_used = chat_messages_used + 1
            WHERE id = %s
            RETURNING chat_messages_used
        """, (user["id"],))
        
        result = cur.fetchone()
        conn.commit()
        cur.close()
        conn.close()
        
        return {"used": result["chat_messages_used"]}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


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
    port = int(os.getenv("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)
