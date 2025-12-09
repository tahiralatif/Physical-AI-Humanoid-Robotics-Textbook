from fastapi import APIRouter, Depends, HTTPException, Body
from sqlalchemy.orm import Session
from pydantic import BaseModel
from typing import List, Optional

from app.database import get_db
# from app.api.auth import get_current_user # Auth optional for now for testing
from app.services.chatbot_service import generate_rag_response
from app.models import ChatMessage, User

router = APIRouter(prefix="/api/chat", tags=["chat"])

class ChatRequest(BaseModel):
    message: str
    session_id: str
    chapter_context: Optional[str] = None

class ChatResponse(BaseModel):
    id: str
    role: str
    content: str
    sources: List[str]

@router.post("/", response_model=ChatResponse)
async def chat(request: ChatRequest, db: Session = Depends(get_db)):
    # 1. Save User Message
    user_msg = ChatMessage(
        session_id=request.session_id,
        role="user",
        content=request.message
    )
    db.add(user_msg)
    db.commit()
    
    # 2. Generate RAG Response
    try:
        result = await generate_rag_response(request.message, request.session_id)
        answer = result["answer"]
        sources = result["sources"]
        
        # 3. Save Assistant Message
        assistant_msg = ChatMessage(
            session_id=request.session_id,
            role="assistant",
            content=answer,
            sources=sources 
        )
        db.add(assistant_msg)
        db.commit()
        db.refresh(assistant_msg)
        
        return ChatResponse(
            id=str(assistant_msg.id),
            role="assistant",
            content=answer,
            sources=sources
        )
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/history/{session_id}")
async def get_history(session_id: str, db: Session = Depends(get_db)):
    messages = db.query(ChatMessage).filter(ChatMessage.session_id == session_id).order_by(ChatMessage.created_at).all()
    return messages