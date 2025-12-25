#!/usr/bin/env python3
"""
Debug script to check Qdrant client methods and configuration
"""
from qdrant_client import QdrantClient
import qdrant_client.http.models as models
from app.core.config import get_settings

# Check the settings
settings = get_settings()
print("QDRANT_URL:", settings.QDRANT_URL)
print("QDRANT_API_KEY:", "SET" if settings.QDRANT_API_KEY else "NOT SET")

# Create client
try:
    if settings.QDRANT_URL and settings.QDRANT_API_KEY:
        print("Creating client with URL and API key...")
        client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            timeout=5
        )
    else:
        print("Creating in-memory client as fallback...")
        client = QdrantClient(":memory:")
    
    print("Client created successfully!")
    print("Client type:", type(client))
    
    # List all public methods
    all_methods = [method for method in dir(client) if not method.startswith('_') and callable(getattr(client, method))]
    search_methods = [m for m in all_methods if 'search' in m.lower() or 'query' in m.lower()]
    
    print("\nAll methods:", all_methods)
    print("\nSearch/query methods:", search_methods)
    
    # Specifically check the search method
    if hasattr(client, 'search'):
        print("\n'client.search' exists!")
        import inspect
        try:
            sig = inspect.signature(client.search)
            print(f"Signature: {sig}")
        except Exception as e:
            print(f"Could not get signature: {e}")
    else:
        print("\n'client.search' does NOT exist!")
    
    # Check the actual error we were getting
    try:
        result = client.search(
            collection_name="test",
            query_vector=[0.1, 0.2, 0.3]
        )
        print("Search test result:", result)
    except AttributeError as e:
        print(f"AttributeError during search call: {e}")
    except Exception as e:
        print(f"Other error during search call: {e}")

except Exception as e:
    print(f"Error creating client: {e}")