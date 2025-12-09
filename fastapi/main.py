from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api import auth, chat
from app.database import engine, Base
from app.qdrant_client import create_collection

# Create tables
Base.metadata.create_all(bind=engine)

# Create Qdrant collection
try:
    create_collection()
except Exception as e:
    print(f"Qdrant collection setup: {e}")

app = FastAPI(title="Physical AI Textbook API")

# CORS configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:3001"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(auth.router)
app.include_router(chat.router)

@app.get("/")
async def root():
    return {"message": "Physical AI Textbook API"}

@app.get("/health")
async def health():
    return {"status": "healthy"}
