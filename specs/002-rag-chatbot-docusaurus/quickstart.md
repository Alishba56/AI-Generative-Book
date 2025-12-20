# Quickstart: RAG-powered Chatbot

This guide provides instructions on how to set up and run the RAG-powered chatbot.

## Prerequisites

- Python 3.11+
- Node.js 18+
- Docusaurus site

## Backend Setup

1.  Navigate to the `backend` directory.
2.  Create a virtual environment: `python -m venv venv`
3.  Activate the virtual environment: `source venv/bin/activate` (on Windows: `venv\Scripts\activate`)
4.  Install the dependencies: `pip install -r requirements.txt`
5.  Run the backend server: `uvicorn app.main:app --reload`

The backend server will be running at `http://localhost:8000`.

## Frontend Setup

1.  Navigate to the `book-content` directory.
2.  Install the dependencies: `npm install`
3.  Run the Docusaurus site: `npm start`

The Docusaurus site will be running at `http://localhost:3000`.

## Usage

1.  Once the backend is running, send a POST request to `http://localhost:8000/ingest` to ingest the documentation.
2.  Open the Docusaurus site in your browser.
3.  Click on the chat widget to open the chat interface.
4.  Ask a question and get a response from the chatbot.
5.  To re-ingest the documentation, send a POST request to `http://localhost:8000/refresh`.
