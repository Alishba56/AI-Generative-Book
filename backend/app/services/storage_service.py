from qdrant_client import QdrantClient, models
from typing import List, Dict, Any

class QdrantStorageService:
    def __init__(self, db_path: str = "local_qdrant", collection_name: str = "docusaurus_docs"):
        """
        Initializes the service to use a local, file-based Qdrant instance.
        """
        self.client = QdrantClient(path=db_path)
        self.collection_name = collection_name
        self._create_collection_if_not_exists()

    def _create_collection_if_not_exists(self):
        try:
            self.client.get_collection(collection_name=self.collection_name)
        except Exception: # Collection does not exist
            self.client.recreate_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE), # bge-small-en-v1.5 has 384 dimensions
            )

    def upsert_chunks(self, chunks: List[Dict[str, Any]]):
        """
        Upserts document chunks into the Qdrant collection.
        Each chunk should be a dictionary matching the DocumentChunk structure.
        """
        points = []
        for chunk in chunks:
            points.append(
                models.PointStruct(
                    id=chunk["chunk_id"],
                    vector=chunk["vector"],
                    payload={"source_file": chunk["source_file"], "content": chunk["content"]}
                )
            )
        self.client.upsert(
            collection_name=self.collection_name,
            wait=True,
            points=points
        )

    def search(self, query_vector: List[float], limit: int = 3) -> List[Dict[str, Any]]:
        """
        Searches for similar document chunks in the Qdrant collection.
        """
        search_result = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=limit,
            with_payload=True
        )
        results = []
        for hit in search_result:
            results.append({
                "chunk_id": hit.id,
                "source_file": hit.payload["source_file"],
                "content": hit.payload["content"],
                "score": hit.score
            })
        return results

qdrant_storage_service = QdrantStorageService()
