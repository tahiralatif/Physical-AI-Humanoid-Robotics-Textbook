#!/usr/bin/env python3
"""
Quick test to check Qdrant connection
"""
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient

load_dotenv()

def test_qdrant_connection():
    url = os.getenv("QDRANT_URL")
    api_key = os.getenv("QDRANT_API_KEY")
    
    print(f"QDRANT_URL: {url}")
    print(f"QDRANT_API_KEY: {api_key[:20]}..." if api_key else "QDRANT_API_KEY: Not set")
    
    if not url or not api_key:
        print("❌ Qdrant credentials missing!")
        return False
    
    try:
        client = QdrantClient(
            url=url,
            api_key=api_key,
            timeout=10
        )
        
        # Test connection
        collections = client.get_collections()
        print(f"✅ Connection successful!")
        print(f"📊 Collections found: {len(collections.collections)}")
        
        for collection in collections.collections:
            print(f"  - {collection.name}")
            
        return True
        
    except Exception as e:
        print(f"❌ Connection failed: {str(e)}")
        return False

if __name__ == "__main__":
    test_qdrant_connection()