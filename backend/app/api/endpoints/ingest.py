from fastapi import APIRouter, HTTPException, status
from app.services.ingestion_service import ingestion_service

router = APIRouter()

@router.post("/ingest", status_code=status.HTTP_200_OK)
async def ingest_documents_endpoint():
    try:
        ingestion_service.ingest_documents()
        return {"message": "Documents ingestion initiated successfully."}
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to ingest documents: {str(e)}"
        )

@router.post("/refresh", status_code=status.HTTP_200_OK)
async def refresh_documents_endpoint():
    try:
        ingestion_service.ingest_documents()
        return {"message": "Documents refresh initiated successfully."}
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to refresh documents: {str(e)}"
        ) 
    
    