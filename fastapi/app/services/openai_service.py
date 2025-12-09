import os
from openai import AsyncOpenAI
from fastapi import HTTPException
from dotenv import load_dotenv

load_dotenv()

# Use the key provided by the user if not in env, but prefer env
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY") 
# Fallback to the key user provided in chat (not recommended for prod but necessary here)
if not GEMINI_API_KEY:
    # Set the key provided by user (Note: In real app, this should be in .env)
    GEMINI_API_KEY = "AIzaSyDEEFYJvvoomvcFVyWt_ml1_eNmsX_qF8c" 

class OpenAIService:
    def __init__(self):
        if GEMINI_API_KEY:
            self.client = AsyncOpenAI(
                api_key=GEMINI_API_KEY,
                base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
            )
            self.chat_model = "gemini-2.0-flash" 
            # Note: For embeddings, Gemini OpenAI compatibility might differ or require specific model
            # Usually text-embedding-004
            self.embedding_model = "text-embedding-004"
        else:
            self.client = None
            print("Warning: GEMINI_API_KEY not found.")

    async def get_embedding(self, text: str) -> list[float]:
        try:
            if not self.client:
                raise Exception("API Key not configured")
                
            response = await self.client.embeddings.create(
                model=self.embedding_model,
                input=text
            )
            return response.data[0].embedding
        except Exception as e:
            print(f"Error generating embedding: {e}")
            raise HTTPException(status_code=500, detail=f"Embedding generation failed: {str(e)}")

    async def generate_chat_response(self, messages: list) -> str:
        try:
            if not self.client:
                raise Exception("API Key not configured")
            
            response = await self.client.chat.completions.create(
                model=self.chat_model,
                messages=messages
            )
            return response.choices[0].message.content
        except Exception as e:
            print(f"Error generating chat response: {e}")
            raise HTTPException(status_code=500, detail=f"Chat generation failed: {str(e)}")

openai_service = OpenAIService()
