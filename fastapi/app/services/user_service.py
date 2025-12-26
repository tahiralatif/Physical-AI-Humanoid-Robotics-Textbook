from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select, update, insert
from app.models.user import UserProfileUpdate, UserProfile
from typing import Optional
import logging

logger = logging.getLogger(__name__)

class UserService:
    async def get_profile(self, db: AsyncSession, user_id: str) -> Optional[dict]:
        # Simple SQL execution since we are using SQLModel/SQLAlchemy lightly here
        from sqlalchemy import text
        result = await db.execute(
            text("SELECT expertise_level, hardware, goals, updated_at FROM profiles WHERE user_id = :user_id"),
            {"user_id": user_id}
        )
        row = result.fetchone()
        if row:
            return {
                "user_id": user_id,
                "expertise_level": row[0],
                "hardware": row[1],
                "goals": row[2],
                "updated_at": row[3]
            }
        return None

    async def update_profile(self, db: AsyncSession, user_id: str, profile_update: UserProfileUpdate):
        from sqlalchemy import text
        # Upsert logic
        existing = await self.get_profile(db, user_id)
        if existing:
            await db.execute(
                text("""
                    UPDATE profiles 
                    SET expertise_level = :exp, hardware = :hw, goals = :goals, updated_at = NOW()
                    WHERE user_id = :user_id
                """),
                {
                    "exp": profile_update.expertise_level,
                    "hw": profile_update.hardware,
                    "goals": profile_update.goals,
                    "user_id": user_id
                }
            )
        else:
            await db.execute(
                text("""
                    INSERT INTO profiles (user_id, expertise_level, hardware, goals)
                    VALUES (:user_id, :exp, :hw, :goals)
                """),
                {
                    "user_id": user_id,
                    "exp": profile_update.expertise_level,
                    "hw": profile_update.hardware,
                    "goals": profile_update.goals
                }
            )
        await db.commit()
        return await self.get_profile(db, user_id)

user_service = UserService()
