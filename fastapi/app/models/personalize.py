from pydantic import BaseModel

class PersonalizeRequest(BaseModel):
    chapter_id: str
    content: str
    user_context: str # e.g. "Beginner Python developer with no hardware"

class PersonalizeResponse(BaseModel):
    personalized_content: str
