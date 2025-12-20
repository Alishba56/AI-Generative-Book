---
id: 0003
title: Define RAG Chatbot Constitution v1.0.0
stage: constitution
date: 2025-12-09
surface: agent
model: gemini
feature: none
branch: main
user: samskproduction
command: /sp.constitution
labels: [constitution, rag, chatbot]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

/sp.constitution You are building a RAG-based Chatbot to integrate inside an existing Docusaurus documentation site.Core principles:1. Chatbot will answer only from the documentation stored inside the RAG system.2. No hallucination — if answer is not found, respond: “This information is not available in the docs.”3. Must support searching across Docusaurus MDX documentation.4. Architecture must allow simple JS/React frontend chat widget inside Docusaurus.5. Backend must use FastAPI and a lightweight vector DB (Qdrant/Chroma).6. Retrieval accuracy more important than creativity.7. Should work offline and inside local environment.8. Handle markdown chunks cleanly (titles, code blocks, text).9. Chat UI must be embed-friendly (iframe or React component).10. Maintain security: no user data stored.

## Response snapshot

Successfully updated the project constitution to version 1.0.0 for the RAG-based Chatbot for Docusaurus.

## Outcome

- ✅ Impact: Established the core principles for the new RAG chatbot project.
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md
- 🔁 Next prompts: none
- 🧠 Reflection: The agent correctly identified that the existing constitution was for a different project and replaced it entirely, correctly versioning the new constitution at 1.0.0.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none
