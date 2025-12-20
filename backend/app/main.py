import asyncio
import sys

# This is a workaround for a bug in uvicorn on Windows
# with some versions of Python.
if sys.platform == "win32":
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api.endpoints import ingest, chat

app = FastAPI(
    title="AI-Native Robotics Textbook RAG API",
    description="API for the RAG chatbot for the AI-Native Robotics textbook.",
    version="0.1.0",
)

# Add CORS middleware
origins = [
    "http://localhost",
    "http://localhost:3000",  # Docusaurus frontend
    "http://127.0.0.1:3000",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routers
app.include_router(ingest.router)
app.include_router(chat.router)


@app.get("/")
async def root():
    return {"message": "RAG Chatbot Backend is running!"}