from fastapi import APIRouter, Depends, HTTPException, status, Response
from sqlalchemy.orm import Session
from pydantic import BaseModel, EmailStr
from app.database import get_db
from app.models import User, UserProfile
from app.auth import get_password_hash, verify_password, create_access_token
import re

router = APIRouter(prefix="/api/auth", tags=["auth"])

class SignupRequest(BaseModel):
    email: EmailStr
    password: str
    python_level: str = None
    ros_experience: str = None
    linux_familiarity: str = None
    hardware_access: str = None
    learning_goal: str = None

class LoginRequest(BaseModel):
    email: EmailStr
    password: str

class AuthResponse(BaseModel):
    access_token: str
    token_type: str = "bearer"
    user_id: int
    email: str

def validate_password(password: str):
    if len(password) < 8:
        raise HTTPException(status_code=400, detail="Password must be at least 8 characters")
    if not re.search(r"[A-Z]", password):
        raise HTTPException(status_code=400, detail="Password must contain uppercase letter")
    if not re.search(r"[a-z]", password):
        raise HTTPException(status_code=400, detail="Password must contain lowercase letter")
    if not re.search(r"\d", password):
        raise HTTPException(status_code=400, detail="Password must contain number")

def classify_expertise(python_level: str, ros_experience: str) -> str:
    if python_level in ["beginner", "none"] or ros_experience in ["none", "beginner"]:
        return "Beginner"
    elif python_level == "advanced" and ros_experience in ["intermediate", "advanced"]:
        return "Expert"
    else:
        return "Intermediate"

@router.post("/signup", response_model=AuthResponse)
async def signup(request: SignupRequest, response: Response, db: Session = Depends(get_db)):
    validate_password(request.password)
    
    existing_user = db.query(User).filter(User.email == request.email).first()
    if existing_user:
        raise HTTPException(status_code=400, detail="Email already registered")
    
    user = User(
        email=request.email,
        password_hash=get_password_hash(request.password)
    )
    db.add(user)
    db.commit()
    db.refresh(user)
    
    expertise_level = classify_expertise(
        request.python_level or "beginner",
        request.ros_experience or "none"
    )
    
    profile = UserProfile(
        user_id=user.id,
        python_level=request.python_level,
        ros_experience=request.ros_experience,
        linux_familiarity=request.linux_familiarity,
        hardware_access=request.hardware_access,
        learning_goal=request.learning_goal,
        expertise_level=expertise_level
    )
    db.add(profile)
    db.commit()
    
    access_token = create_access_token(data={"sub": str(user.id), "email": user.email})
    
    # Set HTTP-only cookie
    response.set_cookie(
        key="access_token",
        value=access_token,
        httponly=True,
        secure=True,
        samesite="lax",
        max_age=60*60*24*7  # 7 days
    )
    
    return AuthResponse(
        access_token=access_token,
        user_id=user.id,
        email=user.email
    )

@router.post("/login", response_model=AuthResponse)
async def login(request: LoginRequest, db: Session = Depends(get_db)):
    user = db.query(User).filter(User.email == request.email).first()
    if not user or not verify_password(request.password, user.password_hash):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password"
        )
    
    access_token = create_access_token(data={"sub": str(user.id), "email": user.email})
    
    return AuthResponse(
        access_token=access_token,
        user_id=user.id,
        email=user.email
    )
