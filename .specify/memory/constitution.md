<!--
Version change: old (0.2.0) → new (1.0.0)
List of modified principles:
  - Replaced all principles to align with the new project: RAG-based Chatbot for Docusaurus.
Added sections: N/A
Removed sections:
  - Constraints
  - Success Criteria
Templates requiring updates:
  - .specify/templates/plan-template.md: ✅ updated (implicit, no direct change needed as it refers to constitution)
  - .specify/templates/spec-template.md: ✅ updated (implicit, no direct change needed as it refers to constitution)
  - .specify/templates/tasks-template.md: ✅ updated (implicit, no direct change needed as it refers to constitution)
  - .gemini/commands/sp.adr.toml: ✅ updated (reviewed, no changes needed)
  - .gemini/commands/sp.analyze.toml: ✅ updated (reviewed, no changes needed)
  - .gemini/commands/sp.checklist.toml: ✅ updated (reviewed, no changes needed)
  - .gemini/commands/sp.clarify.toml: ✅ updated (reviewed, no changes needed)
  - .gemini/commands/sp.constitution.toml: ✅ updated (reviewed, no changes needed)
  - .gemini/commands/sp.git.commit_pr.toml: ✅ updated (reviewed, no changes needed)
  - .gemini/commands/sp.implement.toml: ✅ updated (reviewed, no changes needed)
  - .gemini/commands/sp.phr.toml: ✅ updated (reviewed, no changes needed)
  - .gemini/commands/sp.plan.toml: ✅ updated (reviewed, no changes needed)
  - .gemini/commands/sp.specify.toml: ✅ updated (reviewed, no changes needed)
  - .gemini/commands/sp.tasks.toml: ✅ updated (reviewed, no changes needed)
Follow-up TODOs: N/A
Runtime guidance docs:
  - README.md: ⚠ pending (file not found in current directory structure)
  - docs/quickstart.md: ⚠ pending (file not found in current directory structure)
-->
# RAG-based Chatbot for Docusaurus Constitution

## Core Principles

### I. Documentation-Grounded Responses
The chatbot MUST answer questions using only the documentation stored within the RAG system as its source of truth.

### II. No Hallucination
If the answer to a query is not found in the documentation, the chatbot MUST respond with a definitive statement: “This information is not available in the docs.” It SHALL NOT invent, infer, or provide information from outside sources.

### III. MDX Compatibility
The system MUST support searching and processing Docusaurus MDX documentation, including parsing and handling markdown-specific syntax.

### IV. Simple Frontend Integration
The architecture MUST allow for a simple JavaScript/React frontend chat widget to be integrated within the Docusaurus site.

### V. FastAPI and Lightweight Vector DB Backend
The backend stack is explicitly defined: it MUST use Python with the FastAPI framework and a lightweight, local vector database (such as Qdrant or ChromaDB).

### VI. High-Fidelity Retrieval
The retrieval mechanism's primary goal is accuracy. The system should prioritize finding the correct and relevant passages from the documentation over generating creative or expansive text.

### VII. Local-First and Offline Operation
The entire system MUST be designed to work in an offline, local environment without reliance on external cloud services for its core functionality.

### VIII. Clean Markdown Chunking
The document ingestion process MUST handle markdown content cleanly, preserving the integrity and context of titles, code blocks, lists, and plain text.

### IX. Embeddable Chat UI
The Chat UI MUST be designed to be easily embeddable within the existing Docusaurus site, either as a standalone React component or within an iframe.

### X. User Privacy
The system MUST be secure and stateless regarding user data. No personally identifiable information or chat history should be stored.

## Governance
This Constitution outlines the foundational principles and guidelines for the "RAG-based Chatbot for Docusaurus" project.
- Amendments: Any amendments to this Constitution require a formal proposal, discussion, and approval by the project leads. All changes MUST be documented with a clear rationale and an incremented version.
- Compliance: All project contributions and artifacts MUST comply with the principles and guidelines set forth in this document. Regular reviews will assess adherence.
- Conflict Resolution: In cases of conflict between project practices and this Constitution, the Constitution SHALL take precedence.

**Version**: 1.0.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-09
