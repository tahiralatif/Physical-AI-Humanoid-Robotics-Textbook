from typing import Annotated
from fastapi import Header, HTTPException, status, Depends
from app.core.config import get_settings

settings = get_settings()

async def verify_token(authorization: Annotated[str | None, Header()] = None):
    """
    Verify the Authorization header contains a valid token.
    For now, this is a placeholder that checks for presence.
    In a real Better-Auth setup, you would verify the JWT signature or call the auth server.
    """
    if not authorization:
        # For now, we allow unauthenticated access to basic chat, 
        # but in production you might want to restrict it.
        # Returning None implies anonymous user.
        return None
    
    scheme, _, param = authorization.partition(" ")
    if scheme.lower() != "bearer":
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid authentication credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )
    
    # TODO: Verify 'param' (token) against Better-Auth public key or secret
    return param

UserToken = Annotated[str | None, Depends(verify_token)]
