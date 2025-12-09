import os
import psycopg2
from dotenv import load_dotenv

load_dotenv()

DATABASE_URL = os.getenv("DATABASE_URL")

if not DATABASE_URL:
    print("Error: DATABASE_URL not found in environment variables")
    exit(1)

def run_migrations():
    print(f"Connecting to database...")
    try:
        conn = psycopg2.connect(DATABASE_URL)
        cur = conn.cursor()
        
        # Better Auth Tables
        print("Creating Better Auth tables...")
        cur.execute("""
        CREATE TABLE IF NOT EXISTS "user" (
            id TEXT PRIMARY KEY,
            name TEXT NOT NULL,
            email TEXT NOT NULL UNIQUE,
            "emailVerified" BOOLEAN NOT NULL,
            image TEXT,
            "createdAt" TIMESTAMP NOT NULL,
            "updatedAt" TIMESTAMP NOT NULL
        );
        
        CREATE TABLE IF NOT EXISTS "session" (
            id TEXT PRIMARY KEY,
            "expiresAt" TIMESTAMP NOT NULL,
            "ipAddress" TEXT,
            "userAgent" TEXT,
            "userId" TEXT NOT NULL REFERENCES "user"(id) ON DELETE CASCADE
        );
        
        CREATE TABLE IF NOT EXISTS "account" (
            id TEXT PRIMARY KEY,
            "accountId" TEXT NOT NULL,
            "providerId" TEXT NOT NULL,
            "userId" TEXT NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
            "accessToken" TEXT,
            "refreshToken" TEXT,
            "idToken" TEXT,
            "expiresAt" TIMESTAMP,
            "password" TEXT
        );
        
        CREATE TABLE IF NOT EXISTS "verification" (
            id TEXT PRIMARY KEY,
            identifier TEXT NOT NULL,
            value TEXT NOT NULL,
            "expiresAt" TIMESTAMP NOT NULL
        );
        """)
        
        # Custom User Profiles Table
        print("Creating User Profiles table...")
        cur.execute("""
        CREATE TABLE IF NOT EXISTS user_profiles (
            profile_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            user_id TEXT REFERENCES "user"(id) ON DELETE CASCADE,
            expertise_level VARCHAR(20),
            python_level VARCHAR(20),
            ros_experience VARCHAR(20),
            hardware_access VARCHAR(20),
            learning_goal VARCHAR(20),
            created_at TIMESTAMP DEFAULT NOW(),
            updated_at TIMESTAMP DEFAULT NOW()
        );
        """)
        
        conn.commit()
        cur.close()
        conn.close()
        print("Database initialization completed successfully!")
        
    except Exception as e:
        print(f"Error initializing database: {e}")

if __name__ == "__main__":
    run_migrations()
