from fastapi import APIRouter, HTTPException
from app.models.translate import TranslateRequest, TranslateResponse
from app.services.translate_service import translate_service

router = APIRouter()

@router.post("/translate", response_model=TranslateResponse)
async def translate(request: TranslateRequest):
    try:
        response = await translate_service.translate(request)
        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
