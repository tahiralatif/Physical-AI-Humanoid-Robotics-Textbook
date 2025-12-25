import os
import sys
import asyncio
import re
from pathlib import Path
from typing import List, Dict
import uuid
from qdrant_client.http import models

# Add parent directory to path to import app
current_dir = Path(__file__).resolve().parent
sys.path.append(str(current_dir.parent))

from app.services.openai_service import get_embedding
from app.services.qdrant_service import qdrant_service

DOCS_DIR = (current_dir.parent.parent / "docusaurus/docs").resolve()
COLLECTION_NAME = "textbook_content"

async def read_files(directory: Path) -> List[Dict]:
    documents = []
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith(".md") or file.endswith(".mdx"):
                file_path = Path(root) / file
                content = file_path.read_text(encoding="utf-8")

                # Simple extraction of title from frontmatter or first H1
                title = file_path.stem
                # Try to find title in frontmatter
                fm_match = re.search(r'^---\s+title:\s+(.*?)\s+---', content, re.DOTALL)
                if fm_match:
                    title = fm_match.group(1).strip()

                documents.append({
                    "path": str(file_path.relative_to(DOCS_DIR)),
                    "content": content,
                    "title": title
                })
    return documents

def chunk_text(text: str, max_chars: int = 1500) -> List[str]:
    """
    Split text by headers (##) first, then chunks of max_chars.
    """
    chunks = []
    # Split by H2 headers
    sections = re.split(r'(^##\s+.*$)', text, flags=re.MULTILINE)

    current_chunk = ""

    for section in sections:
        if len(current_chunk) + len(section) > max_chars:
            if current_chunk:
                chunks.append(current_chunk.strip())
            current_chunk = section
        else:
            current_chunk += section

    if current_chunk:
        chunks.append(current_chunk.strip())

    return [c for c in chunks if c.strip()]

async def ingest():
    print(f"Scanning {DOCS_DIR}...")
    documents = await read_files(DOCS_DIR)
    print(f"Found {len(documents)} documents.")

    # Ensure collection exists (using default vector size)
    qdrant_service.create_collection(COLLECTION_NAME, vector_size=768)  # Standard embedding size

    points = []

    for doc in documents:
        print(f"Processing {doc['path']}...")
        chunks = chunk_text(doc["content"])

        for i, chunk in enumerate(chunks):
            # Generate Embedding
            embedding = get_embedding(chunk)  # Using sync function
            if not embedding or len(embedding) == 0:
                print("Skipping chunk due to missing embedding (check API key)")
                continue

            payload = {
                "path": doc["path"],
                "title": doc["title"],
                "chunk_index": i,
                "text": chunk
            }

            point = models.PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding,
                payload=payload
            )
            points.append(point)

            # Batch upsert every 50 chunks
            if len(points) >= 50:
                qdrant_service.upsert(COLLECTION_NAME, points)
                print(f"Upserted batch of {len(points)} chunks")
                points = []

    # Final batch
    if points:
        qdrant_service.upsert(COLLECTION_NAME, points)
        print(f"Upserted final batch of {len(points)} chunks")

    print("Ingestion complete.")

if __name__ == "__main__":
    if not os.path.exists(DOCS_DIR):
        print(f"Error: Docs directory not found at {DOCS_DIR}")
    else:
        asyncio.run(ingest())
