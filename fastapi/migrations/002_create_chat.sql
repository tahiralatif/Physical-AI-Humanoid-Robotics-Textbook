-- Migration 002: Create chat_messages and conversation_sessions tables
-- Physical AI Textbook Database Schema

-- Create conversation_sessions table
CREATE TABLE conversation_sessions (
    session_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(user_id) ON DELETE SET NULL,
    started_at TIMESTAMP DEFAULT NOW(),
    last_message_at TIMESTAMP DEFAULT NOW(),
    message_count INT DEFAULT 0,
    is_active BOOLEAN DEFAULT TRUE
);

-- Create chat_messages table
CREATE TABLE chat_messages (
    message_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL,
    user_id UUID REFERENCES users(user_id) ON DELETE SET NULL,
    role VARCHAR(20) CHECK (role IN ('user', 'assistant')) NOT NULL,
    content TEXT NOT NULL,
    chapter_context VARCHAR(100),
    selected_text TEXT,
    retrieved_chunk_ids TEXT[],
    created_at TIMESTAMP DEFAULT NOW()
);

-- Create indexes
CREATE INDEX idx_session_messages ON chat_messages(session_id, created_at);
CREATE INDEX idx_user_messages ON chat_messages(user_id, created_at);
