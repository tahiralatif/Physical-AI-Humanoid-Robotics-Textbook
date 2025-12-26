"""
Test script to verify the RAG chat functionality
"""
import asyncio
from app.services.openai_service import chat_with_rag

async def test_rag_chat():
    print("Testing RAG chat functionality...")
    
    # Test basic query
    query = "What is humanoid robotics?"
    result = await chat_with_rag(query=query, history=[])
    
    print(f"Query: {query}")
    print(f"Answer: {result['answer']}")
    print(f"Sources: {result['sources']}")
    print("-" * 50)
    
    # Test another query
    query2 = "Explain physical AI systems"
    result2 = await chat_with_rag(query=query2, history=[])
    
    print(f"Query: {query2}")
    print(f"Answer: {result2['answer']}")
    print(f"Sources: {result2['sources']}")
    print("-" * 50)

if __name__ == "__main__":
    asyncio.run(test_rag_chat())