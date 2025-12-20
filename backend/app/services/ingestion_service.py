import os
import uuid
from typing import List, Dict, Any
# import tiktoken
import markdown
import re # Import re for regex operations

from app.services.embedding_service import embedding_service
from app.services.storage_service import qdrant_storage_service

class IngestionService:
    def __init__(self, docs_path: str = "book-content/docs"):
        self.docs_path = docs_path
        # self.tokenizer = tiktoken.get_encoding("cl100k_base")
        self.embedding_service = embedding_service
        self.storage_service = qdrant_storage_service

    def _read_file(self, file_path: str) -> str:
        with open(file_path, 'r', encoding='utf-8') as f:
            return f.read()

    def _strip_markdown(self, content: str) -> str:
        # Convert markdown to plain text
        html = markdown.markdown(content)
        # Remove HTML tags
        plain_text = re.sub(r'<[^>]+>', '', html)
        return plain_text

    def _chunk_text(self, text: str, source_file: str, min_tokens: int = 400, max_tokens: int = 600, overlap: int = 100) -> List[Dict[str, Any]]:
        """
        Chunks text into pieces of min_tokens to max_tokens with a specified overlap.
        """
        # tokens = self.tokenizer.encode(text)
        tokens = text.split()  # approximate tokens as words
        chunks = []
        i = 0
        while i < len(tokens):
            # chunk_tokens = tokens[i : i + max_tokens]
            chunk_tokens = tokens[i : i + max_tokens]
            # chunk_text = self.tokenizer.decode(chunk_tokens)
            chunk_text = ' '.join(chunk_tokens)
            
            # Ensure chunk is not too small, unless it's the last chunk
            if len(chunk_tokens) < min_tokens and (i + len(chunk_tokens)) < len(tokens):
                # If current chunk is too small and not the last, extend it or skip for next iteration
                i += len(chunk_tokens) # Move to the end of this small chunk and try to form a new one
                continue
            
            chunks.append({
                "chunk_id": str(uuid.uuid4()),
                "source_file": source_file,
                "content": chunk_text
            })
            
            # Move index back by overlap to create overlapping chunks
            i += max_tokens - overlap
            if i < 0: # Ensure i doesn't go below 0
                i = 0
        return chunks


    def ingest_documents(self):
        print("Starting document ingestion...")
        all_processed_chunks = []
        for root, _, files in os.walk(self.docs_path):
            for file in files:
                if file.endswith((".md", ".mdx")):
                    file_path = os.path.join(root, file)
                    print(f"Processing {file_path}")
                    content = self._read_file(file_path)
                    
                    # Strip markdown before chunking (or after, depending on preference)
                    # Here, stripping before chunking to ensure plain text chunks
                    plain_text_content = self._strip_markdown(content)

                    file_chunks = self._chunk_text(plain_text_content, file_path)
                    
                    # Generate embeddings for each chunk and add to chunk dict
                    for chunk in file_chunks:
                        chunk["vector"] = self.embedding_service.get_embedding(chunk["content"])
                    
                    all_processed_chunks.extend(file_chunks)
        
        if all_processed_chunks:
            self.storage_service.upsert_chunks(all_processed_chunks)
            print(f"Ingested {len(all_processed_chunks)} chunks into Qdrant.")
        else:
            print("No documents found to ingest.")

ingestion_service = IngestionService()
