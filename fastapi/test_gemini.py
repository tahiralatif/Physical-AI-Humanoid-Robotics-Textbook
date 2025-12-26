"""
Test script to verify the OpenAI/Gemini integration separately
"""
import asyncio
from app.services.openai_service import chat_completion

async def test_gemini_integration():
    print("Testing Gemini API integration...")
    
    # Test basic chat completion
    messages = [
        {"role": "system", "content": "You are a helpful assistant."},
        {"role": "user", "content": "Hello, what is humanoid robotics?"}
    ]
    
    try:
        response = await chat_completion(messages)
        print(f"Response: {response}")
        print("✓ Gemini API integration is working correctly!")
    except Exception as e:
        print(f"✗ Error in Gemini API integration: {e}")

if __name__ == "__main__":
    asyncio.run(test_gemini_integration())