from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean, ForeignKey, JSON
from sqlalchemy.sql import func
from app.database import Base

# Better Auth Tables
class User(Base):
    __tablename__ = "user"  # Better Auth default table name
    
    id = Column(String, primary_key=True, index=True) # Text ID
    name = Column(String, nullable=False)
    email = Column(String, unique=True, index=True, nullable=False)
    emailVerified = Column(Boolean, nullable=False, default=False)
    image = Column(String)
    createdAt = Column(DateTime(timezone=True), nullable=False)
    updatedAt = Column(DateTime(timezone=True), nullable=False)

class Session(Base):
    __tablename__ = "session"
    
    id = Column(String, primary_key=True)
    expiresAt = Column(DateTime(timezone=True), nullable=False)
    ipAddress = Column(String)
    userAgent = Column(String)
    userId = Column(String, ForeignKey("user.id", ondelete="CASCADE"), nullable=False)

class Account(Base):
    __tablename__ = "account"
    
    id = Column(String, primary_key=True)
    accountId = Column(String, nullable=False)
    providerId = Column(String, nullable=False)
    userId = Column(String, ForeignKey("user.id", ondelete="CASCADE"), nullable=False)
    accessToken = Column(String)
    refreshToken = Column(String)
    idToken = Column(String)
    expiresAt = Column(DateTime(timezone=True))
    password = Column(String)

# Custom Tables
class UserProfile(Base):
    __tablename__ = "user_profiles"
    
    profile_id = Column(String, primary_key=True, server_default=func.gen_random_uuid()) # UUID as String or Postgres UUID
    user_id = Column(String, ForeignKey("user.id", ondelete="CASCADE"), unique=True, nullable=False)
    python_level = Column(String)
    ros_experience = Column(String)
    linux_familiarity = Column(String)
    hardware_access = Column(String)
    learning_goal = Column(Text)
    expertise_level = Column(String)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

class ChatMessage(Base):
    __tablename__ = "chat_messages"
    
    id = Column(Integer, primary_key=True, index=True)
    session_id = Column(String, index=True, nullable=False)
    user_id = Column(String, ForeignKey("user.id"), nullable=True)
    role = Column(String, nullable=False)
    content = Column(Text, nullable=False)
    sources = Column(JSON, nullable=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())

class ConversationSession(Base):
    __tablename__ = "conversation_sessions"
    
    id = Column(String, primary_key=True, index=True)
    user_id = Column(String, ForeignKey("user.id"), nullable=True)
    title = Column(String)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

class PersonalizedContent(Base):
    __tablename__ = "personalized_content"
    
    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(String, ForeignKey("user.id"), nullable=False)
    chapter_id = Column(String, nullable=False)
    personalized_markdown = Column(Text, nullable=False)
    expertise_level = Column(String)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    expires_at = Column(DateTime(timezone=True))

class TranslatedChapter(Base):
    __tablename__ = "translated_chapters"
    
    id = Column(Integer, primary_key=True, index=True)
    chapter_id = Column(String, nullable=False)
    language_code = Column(String, nullable=False)
    translated_markdown = Column(Text, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    expires_at = Column(DateTime(timezone=True))

class AgentCallLog(Base):
    __tablename__ = "agent_call_logs"
    
    id = Column(Integer, primary_key=True, index=True)
    agent_name = Column(String, nullable=False)
    task_description = Column(Text)
    input_data = Column(JSON)
    output_data = Column(JSON)
    execution_time_ms = Column(Integer)
    success = Column(Boolean)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
