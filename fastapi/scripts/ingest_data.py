import os
import sys
import asyncio
from pathlib import Path
from qdrant_client.http import models
import uuid

# Add parent directory to path to import app modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.services.openai_service import openai_service
from app.services.qdrant_client import vector_store

DOCS_DIR = r"E:\Documents\quarter_04\hackathon\Physical-AI-Humanoid-Robotics-Textbook\docusaurus\docs"
COLLECTION_NAME = "textbook_chunks"

async def process_file(file_path: Path):
    try:
        content = file_path.read_text(encoding="utf-8")
        # Simple chunking by paragraph/headers for now
        # A more robust splitter would be better for production
        chunks = [c.strip() for c in content.split("\n\n") if c.strip() and len(c) > 50]
        
        chunk_points = []
        for i, chunk in enumerate(chunks):
            # Include some metadata in embedding content for better context
            embedding_content = chunk
            
            vector = await openai_service.get_embedding(embedding_content)
            
            point = models.PointStruct(
                id=str(uuid.uuid4()),
                vector=vector,
                payload={
                    "content": chunk,
                    "source": str(file_path.name),
                    "path": str(file_path),
                    "chunk_index": i
                }
            )
            chunk_points.append(point)
            
        if chunk_points:
            vector_store.upsert_points(COLLECTION_NAME, chunk_points)
            print(f"Uploaded {len(chunk_points)} chunks from {file_path.name}")
            
    except Exception as e:
        print(f"Error processing {file_path}: {e}")

async def main():
    print("Starting ingestion...")
    
    # 1. Ensure collection exists with correct dimension (768 for Gemini text-embedding-004)
    vector_store.create_collection(COLLECTION_NAME, vector_size=768)
    
    # 2. Walk through docs
    tasks = []
    for root, _, files in os.walk(DOCS_DIR):
        for file in files:
            if file.endswith(".md") or file.endswith(".mdx"):
                file_path = Path(root) / file
                tasks.append(process_file(file_path))
    
    # 3. Process concurrently
    # Chunking tasks to avoid rate limits
    chunk_size = 5
    for i in range(0, len(tasks), chunk_size):
        batch = tasks[i:i + chunk_size]
        await asyncio.gather(*batch)
        print(f"Processed batch {i // chunk_size + 1}")

if __name__ == "__main__":
    asyncio.run(main())