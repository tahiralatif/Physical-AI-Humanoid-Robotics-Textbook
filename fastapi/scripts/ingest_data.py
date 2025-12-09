import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from openai import OpenAI
from app.qdrant_client import qdrant_manager
import glob
import re
from dotenv import load_dotenv

load_dotenv()

client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

def chunk_text(text, chunk_size=1000, overlap=200):
    """Split text into overlapping chunks"""
    chunks = []
    start = 0
    while start < len(text):
        end = start + chunk_size
        chunk = text[start:end]
        chunks.append(chunk)
        start = end - overlap
    return chunks

def get_embedding(text):
    """Get OpenAI embedding for text"""
    try:
        response = client.embeddings.create(
            model="text-embedding-3-small",
            input=text
        )
        return response.data[0].embedding
    except Exception as e:
        print(f"Error getting embedding: {e}")
        return None

def process_markdown_files():
    """Process all markdown files in docusaurus/docs"""
    docs_path = "../docusaurus/docs"
    md_files = glob.glob(f"{docs_path}/**/*.md", recursive=True)
    
    all_chunks = []
    
    for file_path in md_files:
        print(f"Processing: {file_path}")
        
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Extract chapter info from path
        relative_path = os.path.relpath(file_path, docs_path)
        chapter_id = relative_path.replace('\\', '/').replace('.md', '')
        
        # Remove frontmatter
        content = re.sub(r'^---.*?---\s*', '', content, flags=re.DOTALL)
        
        # Split into chunks
        text_chunks = chunk_text(content)
        
        for i, chunk in enumerate(text_chunks):
            if len(chunk.strip()) < 50:  # Skip very small chunks
                continue
                
            embedding = get_embedding(chunk)
            if embedding:
                all_chunks.append({
                    "text": chunk,
                    "embedding": embedding,
                    "chapter_id": chapter_id,
                    "section": f"chunk_{i}",
                    "source": relative_path
                })
    
    return all_chunks

if __name__ == "__main__":
    print("Creating Qdrant collection...")
    qdrant_manager.create_collection()
    
    print("Processing markdown files...")
    chunks = process_markdown_files()
    
    print(f"Adding {len(chunks)} chunks to Qdrant...")
    if chunks:
        qdrant_manager.add_chunks(chunks)
        print("Data ingestion complete!")
    else:
        print("No chunks to add")