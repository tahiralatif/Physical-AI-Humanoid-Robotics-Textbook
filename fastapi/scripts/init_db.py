import asyncio
import os
import sys
from pathlib import Path
from sqlalchemy import text

# Add parent directory to path to import app
current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent))

from app.core.db import engine

async def init_db():
    print("Connecting to database to initialize tables...")
    async with engine.begin() as conn:
        # Create profiles table
        print("Creating 'profiles' table...")
        await conn.execute(text("""
            CREATE TABLE IF NOT EXISTS profiles (
                user_id VARCHAR(255) PRIMARY KEY,
                expertise_level VARCHAR(50),
                hardware VARCHAR(100),
                goals TEXT,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
        """))

        # Create personalized_content table
        print("Creating 'personalized_content' table...")
        await conn.execute(text("""
            CREATE TABLE IF NOT EXISTS personalized_content (
                id SERIAL PRIMARY KEY,
                chapter_id VARCHAR(255),
                user_context TEXT,
                content TEXT,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
        """))
        
        # Add index for faster lookup
        await conn.execute(text("""
            CREATE INDEX IF NOT EXISTS idx_personalized_lookup 
            ON personalized_content(chapter_id, user_context);
        """))

    print("Database initialization complete.")

if __name__ == "__main__":
    asyncio.run(init_db())
