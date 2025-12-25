from fastapi import APIRouter
from app.api.endpoints import chat, personalize, translate, user

api_router = APIRouter()
api_router.include_router(chat.router, tags=["chat"])
api_router.include_router(personalize.router, tags=["personalize"])
api_router.include_router(translate.router, tags=["translate"])
api_router.include_router(user.router, prefix="/user", tags=["user"])