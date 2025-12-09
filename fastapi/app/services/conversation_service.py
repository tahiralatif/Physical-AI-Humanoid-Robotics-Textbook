from sqlalchemy.orm import Session
from app.models import ChatMessage, ConversationSession
from datetime import datetime
import uuid

def create_session(db: Session, user_id: int = None) -> str:
    """Create a new conversation session"""
    session_id = str(uuid.uuid4())
    session = ConversationSession(
        id=session_id,
        user_id=user_id,
        title="New Conversation",
        created_at=datetime.utcnow(),
        updated_at=datetime.utcnow()
    )
    db.add(session)
    db.commit()
    return session_id

def save_message(db: Session, session_id: str, role: str, content: str, user_id: int = None, sources: list = None):
    """Save a chat message"""
    message = ChatMessage(
        session_id=session_id,
        user_id=user_id,
        role=role,
        content=content,
        sources=sources,
        created_at=datetime.utcnow()
    )
    db.add(message)
    db.commit()
    
    # Update session timestamp
    session = db.query(ConversationSession).filter(ConversationSession.id == session_id).first()
    if session:
        session.updated_at = datetime.utcnow()
        # Update title from first user message
        if not session.title or session.title == "New Conversation":
            session.title = content[:50] + "..." if len(content) > 50 else content
        db.commit()

def get_conversation_history(db: Session, session_id: str) -> list[dict]:
    """Get conversation history"""
    messages = db.query(ChatMessage).filter(
        ChatMessage.session_id == session_id
    ).order_by(ChatMessage.created_at).all()
    
    return [{"role": msg.role, "content": msg.content} for msg in messages]

def get_user_sessions(db: Session, user_id: int) -> list[dict]:
    """Get all sessions for a user"""
    sessions = db.query(ConversationSession).filter(
        ConversationSession.user_id == user_id
    ).order_by(ConversationSession.updated_at.desc()).all()
    
    return [{
        "id": s.id,
        "title": s.title,
        "created_at": s.created_at.isoformat(),
        "updated_at": s.updated_at.isoformat()
    } for s in sessions]

def delete_session(db: Session, session_id: str):
    """Delete a conversation session and its messages"""
    db.query(ChatMessage).filter(ChatMessage.session_id == session_id).delete()
    db.query(ConversationSession).filter(ConversationSession.id == session_id).delete()
    db.commit()
