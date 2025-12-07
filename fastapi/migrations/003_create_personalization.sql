-- Migration 003: Create personalized_content and translated_chapters tables
-- Physical AI Textbook Database Schema

-- Create personalized_content table
CREATE TABLE personalized_content (
    content_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(user_id) ON DELETE CASCADE,
    chapter_id VARCHAR(100) NOT NULL,
    personalized_version TEXT NOT NULL,
    adjustments_applied JSONB,
    generated_at TIMESTAMP DEFAULT NOW(),
    cache_expires_at TIMESTAMP NOT NULL
);

CREATE INDEX idx_user_chapter_content ON personalized_content(user_id, chapter_id);

-- Create translated_chapters table
CREATE TABLE translated_chapters (
    translation_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chapter_id VARCHAR(100) NOT NULL,
    source_content_hash VARCHAR(64) NOT NULL,
    translated_content TEXT NOT NULL,
    language_code VARCHAR(5) DEFAULT 'ur',
    technical_terms_preserved TEXT[],
    generated_at TIMESTAMP DEFAULT NOW(),
    cache_expires_at TIMESTAMP NOT NULL
);

CREATE INDEX idx_chapter_translation ON translated_chapters(chapter_id, language_code);
