from typing import List, Dict, Any
from app.services.embedding_service import embedding_service
from app.services.storage_service import qdrant_storage_service
from app.services.llm_service import gemini_service

class RAGService:
    def __init__(self):
        self.embedding_service = embedding_service
        self.storage_service = qdrant_storage_service
        self.llm_client = gemini_service

    def _build_rag_prompt(self, query: str, context: str) -> str:
        """
        Constructs a RAG prompt using the query and retrieved context.
        """
        if not context:
            # According to the spec, if answer is not found, respond: “This information is not available in the docs.”
            return "This information is not available in the docs."

        prompt = f"""
        Answer the question based ONLY on the provided documentation.
        If the answer is not in the documentation, say "This information is not available in the docs."
        Be concise.

        Documentation:
        ---
        {context}
        ---

        Question: {query}
        Answer:"""
        return prompt

    def query_rag(self, query: str) -> str:
        # 1. Embed the user's query
        query_vector = self.embedding_service.get_embedding(query)

        # 2. Search Qdrant for relevant chunks
        relevant_chunks = self.storage_service.search(query_vector, limit=3)

        # 3. Construct context from relevant chunks
        context = "\n\n".join([chunk["content"] for chunk in relevant_chunks])
        
        # 4. Build RAG prompt
        rag_prompt = self._build_rag_prompt(query, context)
        
        # If no context, the _build_rag_prompt will return the specific message
        if rag_prompt == "This information is not available in the docs.":
            return rag_prompt

        # 5. Call the LLM with the RAG prompt
        llm_response = self.llm_client.generate_text(rag_prompt)
        return llm_response

    def query_rag_stream(self, query: str):
        # 1. Embed the user's query
        query_vector = self.embedding_service.get_embedding(query)

        # 2. Search Qdrant for relevant chunks
        relevant_chunks = self.storage_service.search(query_vector, limit=3)

        # 3. Construct context from relevant chunks
        context = "\n\n".join([chunk["content"] for chunk in relevant_chunks])
        
        # 4. Build RAG prompt
        rag_prompt = self._build_rag_prompt(query, context)
        
        if rag_prompt == "This information is not available in the docs.":
            yield rag_prompt
            return

        # 5. Call the LLM with the RAG prompt and stream the response
        for token in self.llm_client.generate_text_stream(rag_prompt):
            yield token
        
rag_service = RAGService()