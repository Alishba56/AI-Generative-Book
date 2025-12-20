import pytest
from httpx import AsyncClient
from backend.app.main import app
from unittest.mock import patch, AsyncMock

# --- Test Ingestion Endpoints ---
@pytest.mark.asyncio
async def test_ingest_documents_endpoint():
    with patch('backend.app.api.endpoints.ingest.ingestion_service.ingest_documents', new_callable=AsyncMock) as mock_ingest:
        async with AsyncClient(app=app, base_url="http://test") as ac:
            response = await ac.post("/ingest")
        assert response.status_code == 200
        assert response.json() == {"message": "Documents ingestion initiated successfully."}
        mock_ingest.assert_called_once()

@pytest.mark.asyncio
async def test_refresh_documents_endpoint():
    with patch('backend.app.api.endpoints.ingest.ingestion_service.ingest_documents', new_callable=AsyncMock) as mock_ingest:
        async with AsyncClient(app=app, base_url="http://test") as ac:
            response = await ac.post("/refresh")
        assert response.status_code == 200
        assert response.json() == {"message": "Documents refresh initiated successfully."}
        mock_ingest.assert_called_once()

# --- Test Chat Endpoint ---
@pytest.mark.asyncio
async def test_ask_question_endpoint():
    with patch('backend.app.api.endpoints.chat.rag_service.query_rag', new_callable=AsyncMock) as mock_query_rag:
        mock_query_rag.return_value = "Mocked answer from RAG."
        async with AsyncClient(app=app, base_url="http://test") as ac:
            response = await ac.post("/ask", json={"query": "What is FastAPI?"})
        assert response.status_code == 200
        assert response.json() == {"answer": "Mocked answer from RAG."}
        mock_query_rag.assert_called_once_with("What is FastAPI?")

@pytest.mark.asyncio
async def test_ask_question_endpoint_error():
    with patch('backend.app.api.endpoints.chat.rag_service.query_rag', new_callable=AsyncMock) as mock_query_rag:
        mock_query_rag.side_effect = Exception("RAG error")
        async with AsyncClient(app=app, base_url="http://test") as ac:
            response = await ac.post("/ask", json={"query": "Error question?"})
        assert response.status_code == 500
        assert "Failed to process query: RAG error" in response.json()["detail"]
