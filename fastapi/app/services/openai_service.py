from app.services.qdrant_service import qdrant_service
from app.core.config import get_settings
import os
from dotenv import load_dotenv
import logging
from openai import OpenAI, AsyncOpenAI
import asyncio

load_dotenv()

logger = logging.getLogger(__name__)
settings = get_settings()

# Lazily configure Gemini clients so the API can still start
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

if not GEMINI_API_KEY:
    logger.warning(
        "GEMINI_API_KEY environment variable is not set. "
        "RAG chatbot will return a graceful error instead of crashing the API."
    )
    async_client: AsyncOpenAI | None = None
    sync_client: OpenAI | None = None
else:
    # Initialize clients only when we actually have a key
    async_client = AsyncOpenAI(
        api_key=GEMINI_API_KEY,
        base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
    )

    sync_client = OpenAI(
        api_key=GEMINI_API_KEY,
        base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
    )

def get_embedding(text: str) -> list[float]:
    """Get embedding using Gemini's text-embedding-004 model"""
    if not sync_client:
        logger.error("get_embedding called but GEMINI client is not configured")
        return None
    try:
        response = sync_client.embeddings.create(
            model="text-embedding-004",
            input=text
        )
        embedding = response.data[0].embedding
        logger.info(f"Successfully generated embedding of size {len(embedding)}")
        return embedding
    except Exception as e:
        logger.error(f"Error getting embedding: {str(e)}")
        # Return None instead of fake embedding
        return None

async def chat_completion(messages: list, temperature: float = 0.7, max_tokens: int = 2000) -> str:
    """Generate chat completion using Gemini 2.0 Flash via OpenAI SDK"""
    if not async_client:
        logger.error("chat_completion called but GEMINI client is not configured")
        raise RuntimeError("Gemini API client is not configured")
    try:
        response = await async_client.chat.completions.create(
            model="gemini-2.0-flash-exp",
            messages=messages,
            temperature=temperature,
            max_tokens=max_tokens
        )
        return response.choices[0].message.content
    except Exception as e:
        logger.error(f"Error in chat completion: {str(e)}")
        raise

async def chat_with_rag(
    query: str,
    history: list | None = None,
    collection_name: str = "textbook_content",
    selected_text: str | None = None,
) -> dict:
    """
    RAG-based chat using Qdrant vector search and Gemini API
    
    Args:
        query: User's question
        history: Previous conversation messages
        collection_name: Qdrant collection to search
        selected_text: Optional passage the user highlighted in the textbook
        
    Returns:
        dict with 'answer' and 'sources'
    """
    if history is None:
        history = []

    # Step 1: Get embedding for the query
    logger.info(f"Processing query: {query}")
    query_vector = get_embedding(query)
    
    if query_vector is None:
        logger.error("Failed to generate query embedding")
        return {
            "answer": "Sorry, I'm having trouble processing your question. Please try again.",
            "sources": []
        }

    # Step 2: Search Qdrant for relevant context
    try:
        search_results = qdrant_service.search(
            collection_name=collection_name,
            vector=query_vector,
            limit=5  # Increased to get more context
        )
        logger.info(f"Found {len(search_results)} relevant chunks")
    except Exception as e:
        logger.error(f"Error searching Qdrant: {e}")
        return {
            "answer": "Sorry, I couldn't retrieve relevant information from the textbook.",
            "sources": []
        }

    # Step 3: Build context from search results
    if not search_results:
        return {
            "answer": "I couldn't find relevant information in the textbook to answer your question.",
            "sources": []
        }

    context_parts = []
    sources = []

    # If the user highlighted a passage in the textbook, treat it as
    # high-priority context. This way, the model focuses on explaining
    # exactly what the student selected.
    if selected_text:
        context_parts.append(
            "[Selected Passage]\n"
            f"Content: {selected_text.strip()}\n"
        )
    
    for idx, hit in enumerate(search_results, 1):
        payload = hit.payload
        title = payload.get('title', 'Unknown')
        path = payload.get('path', 'Unknown')
        text = payload.get('text', '').strip()
        score = hit.score
        
        if text:
            context_parts.append(
                f"[Source {idx}] {title} (Score: {score:.3f})\n"
                f"Path: {path}\n"
                f"Content: {text}\n"
            )
            sources.append({
                "title": title,
                "path": path,
                "score": score
            })
    
    context_text = "\n---\n".join(context_parts)

    # Step 4: Create system prompt with context
    system_prompt = f"""You are an expert AI assistant specializing in Physical AI and Humanoid Robotics.

Your task is to answer questions based STRICTLY on the provided textbook content below.

IMPORTANT INSTRUCTIONS:
- Answer ONLY using information from the provided sources
- If the answer is not in the sources, clearly state: "I don't have enough information in the textbook to answer that question."
- Be precise and cite which source number you're using (e.g., "According to Source 1...")
- If multiple sources mention the same thing, acknowledge that
- Provide detailed, technical answers when the information is available
- Use a helpful, educational tone

TEXTBOOK CONTENT:
{context_text}

Now answer the user's question based on this context."""

    # Step 5: Prepare messages for chat completion
    messages = [
        {"role": "system", "content": system_prompt}
    ]

    # Add conversation history (last 4 exchanges = 8 messages)
    if history:
        messages.extend(history[-8:])

    # Add current query
    messages.append({"role": "user", "content": query})

    # Step 6: Generate response
    try:
        response = await chat_completion(messages, temperature=0.3)  # Lower temp for factual answers
        
        logger.info(f"Generated response of length {len(response)}")
        
        return {
            "answer": response,
            "sources": sources,
            "context_used": len(search_results)
        }
    except Exception as e:
        logger.error(f"Error generating response: {str(e)}")
        return {
            "answer": "I encountered an error while generating the response. Please try again.",
            "sources": []
        }

def chat_with_rag_sync(
    query: str,
    history: list | None = None,
    collection_name: str = "textbook_content",
    selected_text: str | None = None,
) -> dict:
    """Synchronous wrapper for RAG chat"""
    return asyncio.run(
        chat_with_rag(
            query=query,
            history=history,
            collection_name=collection_name,
            selected_text=selected_text,
        )
    )


# Example usage function
async def example_usage():
    """Example of how to use the RAG agent"""
    
    # First query
    result1 = await chat_with_rag(
        query="How do humanoid robots maintain balance?",
        history=None,
        selected_text=None,
    )
    
    print("Answer:", result1["answer"])
    print("\nSources:", result1["sources"])
    
    # Follow-up query with history
    history = [
        {"role": "user", "content": "How do humanoid robots maintain balance?"},
        {"role": "assistant", "content": result1["answer"]}
    ]
    
    result2 = await chat_with_rag(
        query="What sensors are used for this?",
        history=history,
        selected_text=None,
    )
    
    print("\n\n--- Follow-up ---")
    print("Answer:", result2["answer"])
    print("\nSources:", result2["sources"])


if __name__ == "__main__":
    # Test the RAG agent
    asyncio.run(example_usage())