import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv

load_dotenv()

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

class VectorStore:
    def __init__(self):
        if QDRANT_URL and QDRANT_API_KEY:
            self.client = QdrantClient(
                url=QDRANT_URL,
                api_key=QDRANT_API_KEY,
            )
        else:
            self.client = None
            print("Warning: Qdrant credentials not found.")

    def create_collection(self, collection_name: str, vector_size: int = 768):
        """Creates a collection if it doesn't exist. Default size 768 for Gemini."""
        if not self.client:
            return

        try:
            self.client.get_collection(collection_name=collection_name)
            print(f"Collection '{collection_name}' already exists.")
        except Exception:
            print(f"Creating collection '{collection_name}' with size {vector_size}...")
            self.client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size, 
                    distance=models.Distance.COSINE
                ),
            )
            print("Collection created successfully.")

    def upsert_points(self, collection_name: str, points: list[models.PointStruct]):
        if not self.client:
            return
        
        self.client.upsert(
            collection_name=collection_name,
            points=points
        )

    def search(self, collection_name: str, query_vector: list[float], limit: int = 5):
        if not self.client:
            return []
            
        return self.client.search(
            collection_name=collection_name,
            query_vector=query_vector,
            limit=limit
        )

vector_store = VectorStore()