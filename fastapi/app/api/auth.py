from fastapi import APIRouter, Depends, HTTPException, status, Request
from sqlalchemy.orm import Session
from pydantic import BaseModel
from typing import Optional
from datetime import datetime

from app.database import get_db
from app.models import User, Session as DbSession, UserProfile

router = APIRouter(prefix="/api/auth", tags=["auth"])

class ProfileCreate(BaseModel):
    python_level: str
    ros_experience: str
    linux_familiarity: str
    hardware_access: str
    learning_goal: str

def classify_expertise(python_level: str, ros_experience: str) -> str:
    if python_level in ["beginner", "none"] or ros_experience in ["none", "beginner"]:
        return "Beginner"
    elif python_level == "advanced" and ros_experience in ["intermediate", "advanced"]:
        return "Expert"
    else:
        return "Intermediate"

async def get_current_user(request: Request, db: Session = Depends(get_db)):
    # Better Auth stores session token in cookie
    # Cookie name is usually 'better-auth.session_token' 
    token = request.cookies.get("better-auth.session_token")
    
    if not token:
        # Fallback to Authorization header if provided
        auth_header = request.headers.get("Authorization")
        if auth_header and auth_header.startswith("Bearer "):
            token = auth_header.split(" ")[1]
    
    if not token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated"
        )
        
    # Verify session in DB
    # Note: If Better Auth uses hashed tokens, we might need verify logic.
    # Assuming direct token match for now based on standard session tables.
    session = db.query(DbSession).filter(DbSession.id == token).first()
    
    if not session:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid session"
        )
        
    if session.expiresAt < datetime.now(session.expiresAt.tzinfo):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Session expired"
        )
        
    user = db.query(User).filter(User.id == session.userId).first()
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found"
        )
        
    return user

@router.post("/profile")
async def create_profile(
    profile: ProfileCreate, 
    user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    # Check if profile exists
    existing_profile = db.query(UserProfile).filter(UserProfile.user_id == user.id).first()
    if existing_profile:
        return {"message": "Profile already exists", "expertise_level": existing_profile.expertise_level}

    expertise_level = classify_expertise(profile.python_level, profile.ros_experience)
    
    new_profile = UserProfile(
        user_id=user.id,
        python_level=profile.python_level,
        ros_experience=profile.ros_experience,
        linux_familiarity=profile.linux_familiarity,
        hardware_access=profile.hardware_access,
        learning_goal=profile.learning_goal,
        expertise_level=expertise_level
    )
    
    db.add(new_profile)
    db.commit()
    db.refresh(new_profile)
    
    return {
        "message": "Profile created successfully",
        "expertise_level": expertise_level,
        "profile_id": str(new_profile.profile_id)
    }

@router.get("/me")
async def get_me(user: User = Depends(get_current_user)):
    return {
        "id": user.id,
        "name": user.name,
        "email": user.email,
        "emailVerified": user.emailVerified
    }
