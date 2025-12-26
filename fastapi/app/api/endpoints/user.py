from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.ext.asyncio import AsyncSession
from app.core.db import get_session
from app.api.deps import verify_token
from app.models.user import UserProfile, UserProfileUpdate
from app.services.user_service import user_service

router = APIRouter()

@router.get("/profile", response_model=UserProfile)
async def get_my_profile(
    db: AsyncSession = Depends(get_session),
    token: str = Depends(verify_token)
):
    if not token:
        raise HTTPException(status_code=401, detail="Authentication required")
    
    # In a real app, you'd decode user_id from token
    # For now, we use the token as a dummy user_id or a fixed test one
    user_id = token 
    
    profile = await user_service.get_profile(db, user_id)
    if not profile:
        raise HTTPException(status_code=404, detail="Profile not found")
    return profile

@router.post("/profile", response_model=UserProfile)
async def update_my_profile(
    profile_update: UserProfileUpdate,
    db: AsyncSession = Depends(get_session),
    token: str = Depends(verify_token)
):
    if not token:
        raise HTTPException(status_code=401, detail="Authentication required")
    
    user_id = token
    profile = await user_service.update_profile(db, user_id, profile_update)
    return profile
