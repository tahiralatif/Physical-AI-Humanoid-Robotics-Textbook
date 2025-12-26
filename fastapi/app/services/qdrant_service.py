from qdrant_client import QdrantClient
from qdrant_client.http import models
from app.core.config import get_settings
import logging

logger = logging.getLogger(__name__)
settings = get_settings()

class QdrantService:
    def __init__(self):
        if settings.QDRANT_URL and settings.QDRANT_API_KEY:
            self.client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
                timeout=30,
                check_compatibility=False  # To avoid the warning about server version
            )
        else:
            logger.warning("Qdrant credentials not found. Vector search will be disabled.")
            self.client = None

    def create_collection(self, collection_name: str, vector_size: int = 768):
        """Create a collection if it doesn't exist."""
        if not self.client:
            return

        if not self.client.collection_exists(collection_name):
            self.client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance.COSINE
                )
            )
            logger.info(f"Created collection: {collection_name}")

    def upsert(self, collection_name: str, points: list[models.PointStruct]):
        if not self.client:
            return

        self.client.upsert(
            collection_name=collection_name,
            points=points
        )

    def search(self, collection_name: str, vector: list[float], limit: int = 5):
        if not self.client:
            return []

        # Check if collection exists before querying
        try:
            collection_exists = self.client.collection_exists(collection_name)
            if not collection_exists:
                logger.warning(f"Collection '{collection_name}' does not exist")
                return []
        except:
            # If we can't check collection existence, still try to query
            pass

        return self.client.query_points(
            collection_name=collection_name,
            query=vector,
            limit=limit
        )

    def search_by_text(self, collection_name: str, query: str, limit: int = 5, get_embedding_func=None):
        """Search using text query by first converting it to embedding"""
        if not self.client:
            return []

        try:
            # Get embedding for the query text - if embedding function is provided
            if get_embedding_func:
                query_vector = get_embedding_func(query)
                if query_vector is None or len(query_vector) == 0:
                    logger.warning("Could not generate embedding for query, performing keyword search instead")
                    # Fallback to keyword search if embedding fails
                    return self.keyword_search(collection_name, query, limit)

                # Perform vector search using the new API
                return self.client.query_points(
                    collection_name=collection_name,
                    query=query_vector,
                    limit=limit
                )
            else:
                logger.warning("No embedding function provided, using keyword search")
                return self.keyword_search(collection_name, query, limit)
        except Exception as e:
            logger.error(f"Error in search_by_text: {str(e)}")
            # Fallback to keyword search
            return self.keyword_search(collection_name, query, limit)

    def keyword_search(self, collection_name: str, query: str, limit: int = 5):
        """Fallback keyword search using Qdrant's full-text capabilities"""
        if not self.client:
            return []

        try:
            # Using scroll API to get records matching certain criteria
            # This is a simple fallback - in a real implementation you might want to
            # implement more sophisticated full-text search
            # Note: Qdrant's MatchText requires the text field to be indexed for full-text search
            # For now, we'll use a simpler approach to find matching records
            hits = self.client.scroll(
                collection_name=collection_name,
                scroll_filter=models.Filter(
                    should=[
                        models.FieldCondition(
                            key="text",
                            match=models.MatchText(text=query)
                        ),
                        models.FieldCondition(
                            key="title",
                            match=models.MatchText(text=query)
                        )
                    ]
                ),
                limit=limit
            )
            # Scroll returns [records, next_offset], we only need the records
            return hits[0] if hits else []
        except:
            # If keyword search also fails, return empty results
            try:
                # Final fallback: simple scroll through all records
                hits = self.client.scroll(
                    collection_name=collection_name,
                    limit=limit
                )
                return hits[0] if hits else []
            except:
                return []

qdrant_service = QdrantService()
