# Tasks: RAG-powered Chatbot for Docusaurus

**Input**: Design documents from `specs/002-rag-chatbot-docusaurus/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create project structure with `backend` and `frontend` directories.
- [ ] T002 Initialize Python project in `backend` with FastAPI and dependencies.
- [ ] T003 Initialize React project in `frontend` for Docusaurus integration.

---

## Phase 2: Foundational (Backend)

**Purpose**: Core backend infrastructure that MUST be complete before ANY user story can be implemented

- [ ] T004 [P] Implement embedding service in `backend/app/services/embedding_service.py` to handle `bge-small` model.
- [ ] T005 [P] Implement storage service for Qdrant in `backend/app/services/storage_service.py`.
- [ ] T006 [P] Implement core RAG service in `backend/app/services/rag_service.py`.
- [ ] T007 [P] Define chat request/response models in `backend/app/models/chat.py`.
- [ ] T008 Setup basic API routing in `backend/app/main.py`.

**Checkpoint**: Foundation ready - user story implementation can now begin.

---

## Phase 3: User Story 1 - Ingest Documentation (Priority: P1) 🎯 MVP

**Goal**: Ingest all documentation from the Docusaurus `/docs` folder so that the chatbot has the information it needs to answer questions.

**Independent Test**: Call the `/ingest` endpoint and verify that the documents are chunked, embedded, and stored in the vector database.

### Implementation for User Story 1

- [ ] T009 [US1] Implement MDX file reader in a new `backend/app/services/ingestion_service.py`.
- [ ] T010 [US1] Implement chunking logic (400-600 tokens) in `backend/app/services/ingestion_service.py`.
- [ ] T011 [US1] Integrate embedding and storage services into `backend/app/services/ingestion_service.py`.
- [ ] T012 [US1] Create `/ingest` endpoint in `backend/app/api/endpoints/ingest.py`.
- [ ] T013 [US1] Create `/refresh` endpoint in `backend/app/api/endpoints/ingest.py` for hot-reloading.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently.

---

## Phase 4: User Story 2 - Ask a Question (Priority: P1)

**Goal**: As a site visitor, I want to ask a question in the chat widget and receive a relevant answer based on the documentation.

**Independent Test**: Send a query to the `/ask` endpoint and verify that the response is relevant to the documentation.

### Implementation for User Story 2

- [ ] T014 [US2] Implement `/ask` endpoint in `backend/app/api/endpoints/chat.py`.
- [ ] T015 [US2] Implement RAG prompt template in `backend/app/services/rag_service.py`.
- [ ] T016 [P] [US2] Create `ChatWidget.jsx` component in `frontend/src/components/ChatWidget/index.tsx`.
- [ ] T017 [P] [US2] Implement floating button UI in `frontend/src/theme/Root.tsx` to launch the chat widget.
- [ ] T018 [US2] Implement `fetch("/ask")` logic in `frontend/src/components/ChatWidget/index.tsx`.
- [ ] T019 [US2] Implement markdown rendering for the response in `frontend/src/components/ChatWidget/index.tsx`.

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T020 Add loading states and error handling to the chat widget.
- [ ] T021 Add unit tests for backend services in `backend/tests/test_services.py`.
- [ ] T022 Add integration tests for API endpoints in `backend/tests/test_api.py`.
- [ ] T023 Validate accuracy of the RAG pipeline with a predefined set of questions.
- [ ] T024 Run `quickstart.md` validation.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately.
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories.
- **User Stories (Phase 3 & 4)**: All depend on Foundational phase completion.
- **Polish (Phase 5)**: Depends on all desired user stories being complete.

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories.
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - Depends on User Story 1 being complete for end-to-end testing.

### Within Each User Story

- Models before services.
- Services before endpoints.
- Core implementation before integration.

### Parallel Opportunities

- Phase 2 tasks can largely be run in parallel.
- `ChatWidget.jsx` creation (T016) and floating button UI (T017) can be done in parallel with backend work for User Story 2.
