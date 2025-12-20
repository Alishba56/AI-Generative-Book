# Research: RAG-powered Chatbot for Docusaurus

This document outlines the technology choices for the RAG-powered chatbot feature.

## Backend Framework

- **Decision**: The backend will be built with Python and FastAPI.
- **Rationale**: FastAPI is a modern, fast (high-performance) web framework for building APIs with Python 3.7+ based on standard Python type hints. Its performance and ease of use make it a good choice for this project.
- **Alternatives considered**: 
  - **Flask**: A popular micro-framework, but FastAPI offers better performance and built-in data validation with Pydantic.
  - **Django**: A full-stack framework, which is overkill for this project's API-only needs.

## Frontend Integration

- **Decision**: The frontend will be a React component integrated into Docusaurus.
- **Rationale**: Docusaurus is built with React. Using a React component for the chat widget is the most idiomatic approach, ensuring maximum compatibility and a consistent user experience.
- **Alternatives considered**:
  - **Plain JavaScript widget**: This would be more framework-agnostic but would require more effort to integrate seamlessly with Docusaurus's React-based architecture.
  - **iframe**: An iframe would isolate the chatbot, but it can lead to a less integrated feel and potential styling issues.

## Vector Database

- **Decision**: Qdrant will be used as the vector database.
- **Rationale**: Qdrant is a high-performance, open-source vector database that can be run locally. This aligns with the "local-first" and "offline operation" principles of the project.
- **Alternatives considered**:
  - **ChromaDB**: Another popular local-first vector database. Qdrant was chosen for its reputation for high performance.
  - **FAISS**: A library for efficient similarity search, but it requires more manual setup and management compared to a full-fledged vector database like Qdrant.

## Embedding Model

- **Decision**: SentenceTransformers with the `bge-small` model will be used for generating embeddings.
- **Rationale**: The `bge-small` model offers a great balance between performance (embedding quality) and resource usage. As a local model, it allows the entire system to run offline, which is a key requirement.
- **Alternatives considered**:
  - **OpenAI Embeddings**: While powerful, using OpenAI's API would violate the "local-first" and "offline operation" principles.
  - **Other Hugging Face Models**: There are many other models available, but `bge-small` is a well-regarded and efficient choice for this type of application.
