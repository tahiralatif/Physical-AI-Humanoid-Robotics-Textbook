from pydantic import BaseModel
from typing import List, Optional

class ChatRequest(BaseModel):
    query: str
    history: Optional[List[dict]] = [] # List of {"role": "user/assistant", "content": "..."}
    selected_text: Optional[str] = None # For context menu queries

class ChatResponse(BaseModel):
    answer: str
    sources: List[str] # List of file paths/titles
