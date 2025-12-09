from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import os
from dotenv import load_dotenv

load_dotenv()

class QdrantService:
    def __init__(self):
        self.client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
        self.collection_name = "textbook_chunks"
    
    def create_collection(self):
        """Create textbook_chunks collection if it doesn't exist"""
        try:
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
            )
            print(f"Collection {self.collection_name} created successfully")
        except Exception as e:
            print(f"Collection might already exist: {e}")
    
    def upsert_chunks(self, chunks_data):
        """Insert textbook chunks with embeddings"""
        points = []
        for i, chunk in enumerate(chunks_data):
            points.append(PointStruct(
                id=i,
                vector=chunk["embedding"],
                payload={
                    "text": chunk["text"],
                    "chapter_id": chunk["chapter_id"],
                    "section": chunk["section"],
                    "source": chunk["source"]
                }
            ))
        
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )
    
    def search_similar(self, query_embedding, limit=5):
        """Search for similar chunks"""
        results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit
        )
        return results

qdrant_service = QdrantService()