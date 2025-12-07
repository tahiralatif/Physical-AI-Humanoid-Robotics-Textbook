-- Migration 001: Create users and user_profiles tables
-- Physical AI Textbook Database Schema

-- Create users table
CREATE TABLE users (
    user_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW(),
    email_verified BOOLEAN DEFAULT FALSE
);

-- Create user_profiles table
CREATE TABLE user_profiles (
    profile_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(user_id) ON DELETE CASCADE,
    expertise_level VARCHAR(20) CHECK (expertise_level IN ('beginner', 'intermediate', 'expert')),
    python_level VARCHAR(20) CHECK (python_level IN ('beginner', 'intermediate', 'expert')),
    ros_experience VARCHAR(20) CHECK (ros_experience IN ('none', 'basic', 'advanced')),
    linux_familiarity VARCHAR(20) CHECK (linux_familiarity IN ('none', 'basic', 'expert')),
    hardware_access VARCHAR(20) CHECK (hardware_access IN ('none', 'jetson', 'rtx', 'cloud')),
    learning_goal VARCHAR(20) CHECK (learning_goal IN ('job', 'startup', 'research', 'hobby')),
    preferred_language VARCHAR(5) DEFAULT 'en' CHECK (preferred_language IN ('en', 'ur')),
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);
