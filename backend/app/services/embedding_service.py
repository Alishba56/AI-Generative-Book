# from sentence_transformers import SentenceTransformer

class EmbeddingService:
    def __init__(self):
        # self.model = SentenceTransformer('BAAI/bge-small-en-v1.5')
        pass

    def get_embedding(self, text: str):
        # return self.model.encode(text, normalize_embeddings=True).tolist()
        return [0.1] * 384  # mock embedding

embedding_service = EmbeddingService()
