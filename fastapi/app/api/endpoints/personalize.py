from fastapi import APIRouter, HTTPException, Depends
from sqlalchemy.ext.asyncio import AsyncSession
from app.core.db import get_session
from app.models.personalize import PersonalizeRequest, PersonalizeResponse
from app.services.personalize_service import personalize_service

router = APIRouter()

@router.post("/personalize", response_model=PersonalizeResponse)
async def personalize(
    request: PersonalizeRequest,
    db: AsyncSession = Depends(get_session)
):
    try:
        response = await personalize_service.personalize(db, request)
        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
