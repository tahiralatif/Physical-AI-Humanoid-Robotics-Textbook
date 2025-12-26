from pydantic import BaseModel
from typing import Optional
from datetime import datetime

class UserProfileBase(BaseModel):
    expertise_level: Optional[str] = None
    hardware: Optional[str] = None
    goals: Optional[str] = None

class UserProfileUpdate(UserProfileBase):
    pass

class UserProfile(UserProfileBase):
    user_id: str
    updated_at: datetime

    class Config:
        from_attributes = True
