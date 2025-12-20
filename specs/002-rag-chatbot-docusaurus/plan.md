# Implementation Plan: RAG-powered Chatbot for Docusaurus

**Branch**: `002-rag-chatbot-docusaurus` | **Date**: 2025-12-09 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/002-rag-chatbot-docusaurus/spec.md`

## Summary

The goal of this feature is to add a RAG-powered chatbot to an existing Docusaurus site. The chatbot will answer questions using the project's documentation, which it will ingest from the `/docs` folder. The system will consist of a FastAPI backend for ingestion and querying, and a React-based chat widget integrated into the Docusaurus frontend.

## Technical Context

**Language/Version**: Python 3.11, TypeScript (React)
**Primary Dependencies**: FastAPI, Docusaurus, React, SentenceTransformers (`bge-small`), Qdrant
**Storage**: Qdrant (local vector database)
**Testing**: pytest, React Testing Library
**Target Platform**: Web
**Project Type**: Web application (frontend/backend)
**Performance Goals**: p95 latency for the `/ask` endpoint must be under 3 seconds.
**Constraints**: The integration must not break the Docusaurus build. The chatbot must be able to work in an offline, local environment.
**Scale/Scope**: The system should handle up to 100 `.mdx` files, each up to 5000 words.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

This plan adheres to the principles defined in the `RAG-based Chatbot for Docusaurus Constitution v1.0.0`:
- **I. Documentation-Grounded Responses**: The RAG pipeline is designed specifically for this.
- **II. No Hallucination**: The prompt engineering will enforce this.
- **III. MDX Compatibility**: The ingestion process will handle `.mdx` files.
- **IV. Simple Frontend Integration**: A React component will be used.
- **V. FastAPI and Lightweight Vector DB Backend**: The plan uses FastAPI and Qdrant.
- **VI. High-Fidelity Retrieval**: The focus is on accuracy.
- **VII. Local-First and Offline Operation**: The components are selected for local operation.
- **VIII. Clean Markdown Chunking**: This will be part of the ingestion pipeline.
- **IX. Embeddable Chat UI**: The React widget will be embeddable.
- **X. User Privacy**: No user data will be stored.

## Project Structure

### Documentation (this feature)

```text
specs/002-rag-chatbot-docusaurus/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
└── tasks.md             # Phase 2 output
```

### Source Code (repository root)

```text
# Option 2: Web application (when "frontend" + "backend" detected)
backend/
├── app/
│   ├── main.py
│   ├── core/
│   │   ├── config.py
│   │   └── security.py
│   ├── api/
│   │   ├── endpoints/
│   │   │   ├── chat.py
│   │   │   └── ingest.py
│   │   └── deps.py
│   ├── services/
│   │   ├── embedding_service.py
│   │   ├── rag_service.py
│   │   └── storage_service.py
│   └── models/
│       └── chat.py
└── tests/
    ├── test_api.py
    └── test_services.py

frontend/
├── src/
│   ├── components/
│   │   └── ChatWidget/
│   │       ├── index.tsx
│   │       └── styles.css
│   └── theme/
│       └── Root.tsx
└── tests/
    └── ChatWidget.test.tsx
```

**Structure Decision**: A frontend/backend monorepo structure is chosen to separate the Docusaurus/React frontend from the Python-based RAG backend. The `frontend` directory will contain the Docusaurus plugin and chat widget, while the `backend` directory will house the FastAPI application.

