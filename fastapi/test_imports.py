try:
    import qdrant_client
    print("qdrant_client imported")
    import openai
    print("openai imported")
    from dotenv import load_dotenv
    print("dotenv imported")
except ImportError as e:
    print(f"ImportError: {e}")
except Exception as e:
    print(f"Error: {e}")
