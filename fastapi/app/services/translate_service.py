from app.services.openai_service import chat_completion
from app.models.translate import TranslateRequest, TranslateResponse
import logging

logger = logging.getLogger(__name__)

class TranslateService:
    async def translate(self, request: TranslateRequest) -> TranslateResponse:
        system_prompt = f"""You are a technical translator.
Translate the following Markdown content to {request.target_lang}.
CRITICAL RULES:
1. PRESERVE all code blocks exactly as they are. Do not translate code.
2. PRESERVE all variable names, file paths, and technical terms (e.g. Node, Topic, ROS 2, Middleware) in English.
3. Only translate the explanatory text and headers.
4. Keep the Markdown formatting intact.
"""
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": request.content}
        ]

        try:
            translated = await chat_completion(messages)
            if not translated or "I'm sorry, I'm having trouble" in translated:
                logger.warning("Translation service is unavailable, returning original content")
                return TranslateResponse(translated_content=request.content)
            return TranslateResponse(translated_content=translated)
        except Exception as e:
            logger.error(f"Translation error: {str(e)}")
            # In case of error, return the original content
            return TranslateResponse(translated_content=request.content)

translate_service = TranslateService()
