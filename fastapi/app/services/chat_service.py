from app.services.qdrant_service import qdrant_service
from app.services.openai_service import chat_with_rag
from app.models.chat import ChatRequest, ChatResponse
import logging

logger = logging.getLogger(__name__)

class ChatService:
    async def chat(self, request: ChatRequest) -> ChatResponse:
        try:
            # Call the RAG service to handle the query.
            # We now pass the selected_text so it becomes part of the RAG context.
            result = await chat_with_rag(
                query=request.query,
                history=request.history,
                selected_text=request.selected_text,
            )

            # Ensure the response is properly formatted
            answer = result.get("answer", "I'm sorry, I couldn't process your request.")
            sources = result.get("sources", [])

            # Sanitize the response to prevent any encoding issues
            answer = str(answer) if answer else "I'm sorry, I couldn't process your request."
            sources = [str(source) for source in sources if source is not None]

            return ChatResponse(
                answer=answer,
                sources=sources
            )
        except Exception as e:
            logger.error(f"Error in chat service: {str(e)}")
            return ChatResponse(
                answer="I'm sorry, I'm having trouble processing your request right now.",
                sources=[]
            )

chat_service = ChatService()
