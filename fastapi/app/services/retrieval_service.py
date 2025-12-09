from openai import OpenAI
from app.qdrant_client import qdrant_manager
import os

client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

class RetrievalService:
    def __init__(self):
        self.qdrant = qdrant_manager
    
    def get_embedding(self, text: str):
        """Get embedding for query text"""
        response = client.embeddings.create(
            model="text-embedding-3-small",
            input=text
        )
        return response.data[0].embedding
    
    def search_relevant_chunks(self, query: str, limit: int = 5):
        """Search for relevant text chunks"""
        query_embedding = self.get_embedding(query)
        results = self.qdrant.search(query_embedding, limit=limit)
        
        chunks = []
        for result in results:
            chunks.append({
                "text": result.payload["text"],
                "chapter_id": result.payload["chapter_id"],
                "section": result.payload["section"],
                "source": result.payload["source"],
                "score": result.score
            })
        
        return chunks

retrieval_service = RetrievalService()