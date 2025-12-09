from openai import OpenAI
from app.services.retrieval_service import retrieval_service
import os

client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

class ChatbotService:
    def __init__(self):
        self.retrieval = retrieval_service
    
    def generate_response(self, query: str, conversation_history: list = None):
        """Generate chatbot response with RAG"""
        
        # Get relevant chunks
        relevant_chunks = self.retrieval.search_relevant_chunks(query, limit=5)
        
        if not relevant_chunks:
            return {
                "response": "I couldn't find relevant information in the textbook to answer your question. Please try rephrasing or ask about topics covered in the Physical AI and Robotics course.",
                "sources": [],
                "can_answer": False
            }
        
        # Build context from chunks
        context = "\n\n".join([
            f"Source: {chunk['source']}\n{chunk['text']}" 
            for chunk in relevant_chunks
        ])
        
        # Build conversation history
        messages = [
            {
                "role": "system",
                "content": f"""You are a helpful AI assistant for the Physical AI & Humanoid Robotics textbook. 
                
Use the following context from the textbook to answer questions:

{context}

Guidelines:
- Only answer questions based on the provided context
- If the context doesn't contain relevant information, say so clearly
- Always cite sources by mentioning the chapter/section
- Be concise but thorough
- Use technical terms appropriately for the audience level"""
            }
        ]
        
        # Add conversation history
        if conversation_history:
            messages.extend(conversation_history[-10:])  # Last 10 messages
        
        messages.append({"role": "user", "content": query})
        
        try:
            response = client.chat.completions.create(
                model="gpt-4o-mini",
                messages=messages,
                max_tokens=500,
                temperature=0.7
            )
            
            return {
                "response": response.choices[0].message.content,
                "sources": [
                    {
                        "chapter_id": chunk["chapter_id"],
                        "source": chunk["source"],
                        "score": chunk["score"]
                    }
                    for chunk in relevant_chunks
                ],
                "can_answer": True
            }
            
        except Exception as e:
            return {
                "response": f"Sorry, I encountered an error: {str(e)}",
                "sources": [],
                "can_answer": False
            }

chatbot_service = ChatbotService()