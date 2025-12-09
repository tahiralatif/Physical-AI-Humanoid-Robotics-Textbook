from app.services.openai_service import openai_service
from app.services.qdrant_client import vector_store

COLLECTION_NAME = "textbook_chunks"

async def generate_rag_response(query: str, session_id: str, history: list = None):
    # 1. Generate embedding for query
    query_vector = await openai_service.get_embedding(query)
    
    # 2. Retrieve relevant chunks from Qdrant
    search_results = vector_store.search(COLLECTION_NAME, query_vector, limit=3)
    
    # 3. Construct context
    context_text = ""
    sources = []
    
    for result in search_results:
        payload = result.payload
        content = payload.get("content", "")
        source = payload.get("source", "Unknown")
        context_text += f"\n---\nSource: {source}\nContent: {content}\n"
        if source not in sources:
            sources.append(source)
            
    # 4. Construct Prompt
    system_prompt = """You are an AI assistant for a Physical AI and Humanoid Robotics textbook.
    Answer the user's question based ONLY on the provided context.
    If the answer is not in the context, say "I couldn't find the answer in the textbook."
    Include citations to the source chapters where appropriate.
    """
    
    # 5. Generate Response (OpenAI Chat Format)
    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": f"Context:\n{context_text}\n\nUser Question: {query}"}
    ]
    
    answer = await openai_service.generate_chat_response(messages)
    
    return {
        "answer": answer,
        "sources": sources,
        "context_used": context_text
    }