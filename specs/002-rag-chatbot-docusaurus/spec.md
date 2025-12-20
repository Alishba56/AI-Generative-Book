# Feature Specification: RAG-powered Chatbot for Docusaurus

**Feature Branch**: `002-rag-chatbot-docusaurus`  
**Created**: 2025-12-09  
**Status**: Draft  
**Input**: User description: "Goal:Add a RAG-powered chatbot to an existing Docusaurus site, which can answer questions using project documentation.Functional Requirements:1. AI bot loads documentation from the Docusaurus /docs folder.2. System chunks, embeds, and stores MDX files.3. Chat widget sends user queries to backend.4. Backend retrieves relevant MDX chunks.5. LLM produces grounded answers using strict RAG prompting.Technical Requirements:Frontend:- Docusaurus React plugin- Chat bubble widget (Icon → Open Chat)- Fetch API call to FastAPI backendBackend:- FastAPI- SentenceTransformer embeddings (bge-small)- Vector DB: Qdrant local- RAG prompt template- API endpoints: /ingest, /askConstraints:- Must not break Docusaurus build.- Chatbot loads only after docs have compiled.- Should support markdown formatting in responses."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ingest Documentation (Priority: P1)

As a site administrator, I want to ingest all the documentation from the Docusaurus `/docs` folder so that the chatbot has the information it needs to answer questions.

**Why this priority**: This is the foundational step that makes the chatbot useful.

**Independent Test**: This can be tested by calling the `/ingest` endpoint and verifying that the documents are chunked, embedded, and stored in the vector database.

**Acceptance Scenarios**:

1. **Given** a Docusaurus site with `.mdx` files in the `/docs` folder, **When** the `/ingest` endpoint is called, **Then** the vector database is populated with document chunks.
2. **Given** an empty `/docs` folder, **When** the `/ingest` endpoint is called, **Then** the system handles the situation gracefully without errors.

---

### User Story 2 - Ask a Question (Priority: P1)

As a site visitor, I want to ask a question in the chat widget and receive a relevant answer based on the documentation.

**Why this priority**: This is the core functionality of the chatbot from a user's perspective.

**Independent Test**: This can be tested by sending a query to the `/ask` endpoint and verifying that the response is relevant to the documentation.

**Acceptance Scenarios**:

1. **Given** an ingested documentation set, **When** a user asks a question that is answered in the docs, **Then** the chatbot returns a relevant, grounded answer.
2. **Given** an ingested documentation set, **When** a user asks a question that is not in the docs, **Then** the chatbot responds that the information is not available.

---

### Edge Cases

- What happens when the `/docs` directory does not exist or is empty during ingestion?
- How does the system handle a query to `/ask` before any documents have been ingested?
- How does the system handle very long queries or queries with special characters?
- What is the behavior when the vector search returns no relevant document chunks?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The system MUST load all `.mdx` documentation from a specified Docusaurus `/docs` folder.
- **FR-002**: The system MUST chunk the `.mdx` files into smaller, manageable pieces suitable for embedding.
- **FR-003**: The system MUST generate vector embeddings for each document chunk using a SentenceTransformer model.
- **FR-004**: The system MUST store the document chunks and their corresponding embeddings in a local Qdrant vector database.
- **FR-005**: The frontend MUST provide a chat widget, initially appearing as an icon, that opens a chat interface when clicked.
- **FR-006**: The chat widget MUST send user queries to a FastAPI backend.
- **FR-007**: The backend MUST provide an `/ingest` endpoint to trigger the documentation loading process.
- **FR-008**: The backend MUST provide an `/ask` endpoint that accepts a user query.
- **FR-009**: Upon receiving a query, the backend MUST retrieve the most relevant document chunks from the vector database.
- **FR-010**: The backend MUST use a strict RAG prompt template to combine the user's query and the retrieved chunks.
- **FR-011**: The system MUST generate a final answer using an LLM, grounded by the retrieved context.
- **FR-012**: The chatbot's response MUST support markdown formatting.
- **FR-013**: The chatbot integration MUST NOT break the Docusaurus build process.

### Key Entities *(include if feature involves data)*

- **DocumentChunk**: Represents a piece of text extracted from a larger `.mdx` document.
  - `chunk_id`: Unique identifier for the chunk.
  - `source_file`: The original file path of the document.
  - `content`: The text content of the chunk.
  - `vector`: The numerical vector embedding of the content.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: The `/ingest` process can successfully process 100 `.mdx` files, each containing up to 5000 words, in under 5 minutes on a standard developer machine.
- **SC-002**: For a set of 20 predefined questions with known answers in the documentation, the chatbot MUST provide the correct answer for at least 18 of them (90% accuracy).
- **SC-003**: The p95 latency for the `/ask` endpoint, from receiving the query to returning the final answer, MUST be under 3 seconds.
- **SC-004**: The chat widget MUST load within 500ms and have no discernible impact on the main Docusaurus site's Lighthouse performance score.
