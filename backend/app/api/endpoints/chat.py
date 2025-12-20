from fastapi import APIRouter, HTTPException, status
from fastapi.responses import StreamingResponse
import logging
from app.models.chat import ChatRequest
from app.services.rag_service import rag_service
import google.generativeai as genai
from app.services.llm_service import gemini_service

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api", tags=["chat"])


@router.post("/chat")
async def chat_endpoint(request: ChatRequest):
    """
    Handles a chat request and streams the response back.
    """
    try:
        # Return a streaming response
        return StreamingResponse(
            rag_service.query_rag_stream(request.query),
            media_type="text/event-stream",
        )
    except Exception as e:
        logger.exception("Error during chat streaming")
        # Note: In a streaming response, you can't easily raise an HTTPException
        # after the headers have been sent. Proper error handling would involve
        # streaming an error message, but for now, we log it.
        # A simple fallback could be to return a non-streaming error response
        # if the exception happens before the stream starts.
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to process query: {str(e)}",
        )


@router.get("/test-llm")
async def test_llm():
    """
    A test endpoint to verify the LLM service is working.
    """
    try:
        # List available models
        models = genai.list_models()
        available_models = [
            model.name
            for model in models
            if "generateContent" in model.supported_generation_methods
        ]
        response = gemini_service.generate_text("Say 'Hello, world!'")
        return {"status": "success", "response": response, "available_models": available_models}
    except Exception as e:
        try:
            models = genai.list_models()
            available_models = [
                model.name
                for model in models
                if "generateContent" in model.supported_generation_methods
            ]
        except:
            available_models = []
        return {"status": "error", "message": str(e), "available_models": available_models}
