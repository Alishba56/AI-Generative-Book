import pytest
from unittest.mock import MagicMock, patch
import uuid

# Import the services to be tested
from backend.app.services.embedding_service import EmbeddingService
from backend.app.services.storage_service import QdrantStorageService
from backend.app.services.rag_service import RAGService
from backend.app.services.ingestion_service import IngestionService

# --- Test EmbeddingService ---
def test_embedding_service_get_embedding():
    service = EmbeddingService()
    text = "test sentence"
    embedding = service.get_embedding(text)
    assert isinstance(embedding, list)
    assert len(embedding) == 384 # bge-small-en-v1.5 produces 384-dim embeddings
    assert all(isinstance(x, float) for x in embedding)

# --- Test QdrantStorageService ---
@pytest.fixture
def mock_qdrant_client():
    with patch('qdrant_client.QdrantClient') as MockClient:
        mock_client_instance = MockClient.return_value
        yield mock_client_instance

def test_qdrant_storage_service_init(mock_qdrant_client):
    service = QdrantStorageService(collection_name="test_collection")
    mock_qdrant_client.recreate_collection.assert_called_once_with(
        collection_name="test_collection",
        vectors_config=MagicMock(size=384, distance=MagicMock())
    )

def test_qdrant_storage_service_upsert_chunks(mock_qdrant_client):
    service = QdrantStorageService()
    chunks = [
        {"chunk_id": str(uuid.uuid4()), "vector": [0.1]*384, "source_file": "doc1.md", "content": "chunk1"},
        {"chunk_id": str(uuid.uuid4()), "vector": [0.2]*384, "source_file": "doc2.md", "content": "chunk2"},
    ]
    service.upsert_chunks(chunks)
    mock_qdrant_client.upsert.assert_called_once()
    assert len(mock_qdrant_client.upsert.call_args[1]['points']) == 2

def test_qdrant_storage_service_search(mock_qdrant_client):
    service = QdrantStorageService()
    query_vector = [0.5]*384
    mock_qdrant_client.search.return_value = [
        MagicMock(id="id1", payload={"source_file": "doc1.md", "content": "chunk1"}, score=0.9),
        MagicMock(id="id2", payload={"source_file": "doc2.md", "content": "chunk2"}, score=0.8),
    ]
    results = service.search(query_vector)
    mock_qdrant_client.search.assert_called_once_with(
        collection_name=service.collection_name,
        query_vector=query_vector,
        limit=3,
        with_payload=True
    )
    assert len(results) == 2
    assert results[0]["chunk_id"] == "id1"
    assert results[0]["score"] == 0.9

# --- Test RAGService ---
@pytest.fixture
def mock_rag_dependencies():
    with patch('backend.app.services.embedding_service.embedding_service') as mock_embed_service, \
         patch('backend.app.services.storage_service.qdrant_storage_service') as mock_storage_service:
        yield mock_embed_service, mock_storage_service

def test_rag_service_query_rag_with_context(mock_rag_dependencies):
    mock_embed_service, mock_storage_service = mock_rag_dependencies
    mock_embed_service.get_embedding.return_value = [0.1]*384
    mock_storage_service.search.return_value = [
        {"content": "Relevant document content 1."},
        {"content": "Relevant document content 2."},
    ]
    
    service = RAGService()
    query = "What is this about?"
    response = service.query_rag(query)
    
    assert "Relevant document content 1." in response
    assert "What is this about?" in response
    assert "This information is not available in the docs." not in response

def test_rag_service_query_rag_no_context(mock_rag_dependencies):
    mock_embed_service, mock_storage_service = mock_rag_dependencies
    mock_embed_service.get_embedding.return_value = [0.1]*384
    mock_storage_service.search.return_value = [] # No relevant chunks found
    
    service = RAGService()
    query = "Non-existent question?"
    response = service.query_rag(query)
    
    assert response == "This information is not available in the docs."

# --- Test IngestionService ---
@pytest.fixture
def mock_ingestion_dependencies():
    with patch('os.walk') as mock_os_walk, \
         patch('builtins.open', MagicMock()), \
         patch('backend.app.services.embedding_service.embedding_service') as mock_embed_service, \
         patch('backend.app.services.storage_service.qdrant_storage_service') as mock_storage_service, \
         patch('tiktoken.get_encoding', MagicMock(return_value=MagicMock(encode=lambda x: [1,2,3], decode=lambda x: "chunk_text"))):
        
        # Configure os.walk to return a dummy file structure
        mock_os_walk.return_value = [
            ('/path/to/docs', [], ['doc1.md', 'doc2.mdx']),
        ]
        
        mock_embed_service.get_embedding.return_value = [0.1]*384
        yield mock_embed_service, mock_storage_service

def test_ingestion_service_load_and_chunk_documents(mock_ingestion_dependencies):
    mock_embed_service, mock_storage_service = mock_ingestion_dependencies
    
    service = IngestionService(docs_path="/path/to/docs")
    service.ingest_documents()
    
    # Assert that upsert_chunks was called with processed chunks
    mock_storage_service.upsert_chunks.assert_called_once()
    chunks_arg = mock_storage_service.upsert_chunks.call_args[0][0]
    assert len(chunks_arg) >= 2 # At least 2 chunks from 2 files
    assert "vector" in chunks_arg[0]
    assert "chunk_id" in chunks_arg[0]
    
def test_ingestion_service_strip_markdown():
    service = IngestionService()
    markdown_text = "# Hello\n\n**World**\n\n```python\nprint('hello')\n```"
    plain_text = service._strip_markdown(markdown_text)
    assert "Hello" in plain_text
    assert "World" in plain_text
    assert "print('hello')" in plain_text
    assert "#" not in plain_text
    assert "**" not in plain_text
    assert "```" not in plain_text

def test_ingestion_service_no_documents_found(mock_ingestion_dependencies):
    mock_embed_service, mock_storage_service = mock_ingestion_dependencies
    
    # Configure os.walk to return no files
    mock_ingestion_dependencies[0].return_value = []
    
    service = IngestionService(docs_path="/path/to/empty_docs")
    service.ingest_documents()
    
    mock_storage_service.upsert_chunks.assert_not_called()
