from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from pydantic import BaseModel
from app.database import get_db
from app.models import ChatMessage, ConversationSession
from app.services.chatbot_service import chatbot_service
from app.auth import get_current_user
import uuid
from datetime import datetime
from typing import Optional, List

router = APIRouter(prefix="/api/chat", tags=["chat"])

class ChatRequest(BaseModel):
    message: str
    session_id: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    session_id: str
    sources: List[dict]
    can_answer: bool

class MessageHistory(BaseModel):
    role: str
    content: str
    created_at: datetime

@router.post("/", response_model=ChatResponse)
async def send_message(
    request: ChatRequest, 
    db: Session = Depends(get_db),
    current_user: dict = Depends(get_current_user)
):
    # Get or create session
    session_id = request.session_id or str(uuid.uuid4())
    user_id = int(current_user["sub"]) if current_user else None
    
    # Get conversation history
    history_messages = db.query(ChatMessage).filter(
        ChatMessage.session_id == session_id
    ).order_by(ChatMessage.created_at.desc()).limit(10).all()
    
    conversation_history = [
        {"role": msg.role, "content": msg.content}
        for msg in reversed(history_messages)
    ]
    
    # Generate response
    result = chatbot_service.generate_response(
        request.message, 
        conversation_history
    )
    
    # Save user message
    user_message = ChatMessage(
        session_id=session_id,
        user_id=user_id,
        role="user",
        content=request.message
    )
    db.add(user_message)
    
    # Save bot response
    bot_message = ChatMessage(
        session_id=session_id,
        user_id=user_id,
        role="assistant",
        content=result["response"],
        sources=result["sources"]
    )
    db.add(bot_message)
    
    # Update or create session
    session = db.query(ConversationSession).filter(
        ConversationSession.id == session_id
    ).first()
    
    if not session:
        session = ConversationSession(
            id=session_id,
            user_id=user_id,
            title=request.message[:50] + "..." if len(request.message) > 50 else request.message
        )
        db.add(session)
    
    db.commit()
    
    return ChatResponse(
        response=result["response"],
        session_id=session_id,
        sources=result["sources"],
        can_answer=result["can_answer"]
    )

@router.get("/history/{session_id}")
async def get_chat_history(
    session_id: str,
    db: Session = Depends(get_db)
):
    messages = db.query(ChatMessage).filter(
        ChatMessage.session_id == session_id
    ).order_by(ChatMessage.created_at).all()
    
    return [
        MessageHistory(
            role=msg.role,
            content=msg.content,
            created_at=msg.created_at
        )
        for msg in messages
    ]

@router.delete("/history/{session_id}")
async def clear_chat_history(
    session_id: str,
    db: Session = Depends(get_db)
):
    db.query(ChatMessage).filter(
        ChatMessage.session_id == session_id
    ).delete()
    
    db.query(ConversationSession).filter(
        ConversationSession.id == session_id
    ).delete()
    
    db.commit()
    return {"message": "Chat history cleared"}

@router.get("/sessions")
async def get_chat_sessions(
    db: Session = Depends(get_db),
    current_user: dict = Depends(get_current_user)
):
    user_id = int(current_user["sub"])
    
    sessions = db.query(ConversationSession).filter(
        ConversationSession.user_id == user_id
    ).order_by(ConversationSession.updated_at.desc()).all()
    
    return sessions