from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import text
from app.services.openai_service import chat_completion
from app.models.personalize import PersonalizeRequest, PersonalizeResponse
import logging

logger = logging.getLogger(__name__)

class PersonalizeService:
    async def personalize(self, db: AsyncSession, request: PersonalizeRequest) -> PersonalizeResponse:
        # 1. Check Cache
        try:
            result = await db.execute(
                text("SELECT content FROM personalized_content WHERE chapter_id = :cid AND user_context = :ctx"),
                {"cid": request.chapter_id, "ctx": request.user_context}
            )
            row = result.fetchone()
            if row:
                return PersonalizeResponse(personalized_content=row[0])

            # 2. Generate if Cache Miss
            system_prompt = f"""You are an expert technical editor.
Your task is to rewrite the provided technical content to match the user's background: "{request.user_context}".
- If they are a beginner, simplify explanations and use analogies.
- If they are an expert, be concise and focus on advanced concepts.
- If they lack hardware, emphasize simulation (Gazebo/Isaac).
- PRESERVE all code blocks and technical facts. Do not remove code.
"""
            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": request.content}
            ]

            rewritten = await chat_completion(messages)

            # 3. Store in Cache
            if rewritten and "I'm sorry, I'm having trouble" not in rewritten:
                await db.execute(
                    text("INSERT INTO personalized_content (chapter_id, user_context, content) VALUES (:cid, :ctx, :content)"),
                    {"cid": request.chapter_id, "ctx": request.user_context, "content": rewritten}
                )
                await db.commit()

            # Return content even if rewriting failed
            return PersonalizeResponse(personalized_content=rewritten if rewritten and "I'm sorry, I'm having trouble" not in rewritten else request.content)
        except Exception as e:
            logger.error(f"Personalization error: {str(e)}")
            # On error, return original content
            return PersonalizeResponse(personalized_content=request.content)

personalize_service = PersonalizeService()
