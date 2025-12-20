# Data Model: RAG-powered Chatbot

This document defines the data models for the RAG-powered chatbot.

## Entities

### DocumentChunk

Represents a piece of text extracted from a larger `.mdx` document. This is the core entity that will be stored in the vector database.

**Fields**:

| Field       | Type   | Description                                         |
|-------------|--------|-----------------------------------------------------|
| `chunk_id`  | String | A unique identifier for the chunk (e.g., a UUID).   |
| `source_file`| String | The original file path of the document.             |
| `content`   | String | The text content of the chunk.                      |
| `vector`    | Array  | The numerical vector embedding of the `content`.    |
