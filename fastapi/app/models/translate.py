from pydantic import BaseModel

class TranslateRequest(BaseModel):
    content: str
    target_lang: str = "Urdu"

class TranslateResponse(BaseModel):
    translated_content: str
