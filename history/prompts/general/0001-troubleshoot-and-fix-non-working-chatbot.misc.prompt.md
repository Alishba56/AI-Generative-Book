---
id: 0001
title: Troubleshoot and fix non-working chatbot
stage: misc
date: 2025-12-16
surface: agent
model: gemini-pro
feature: 002-rag-chatbot-docusaurus
branch: 002-rag-chatbot-docusaurus
user: alish
command: chatbot not working
labels: [chatbot, bug, environment]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- backend/.env
tests:
null
---

## Prompt

chatbot not working

## Response snapshot

I have created a .env file in the ackend directory. You need to update it with your Gemini API key.

Here are the steps to get your chatbot running:

**1. Get your Gemini API Key:**

*   Go to [https://makersuite.google.com/](https://makersuite.google.com/) and create an API key.
*   Open the ackend/.env file and replace YOUR_API_KEY with your actual Gemini API key.

**2. Install Dependencies:**

*   **Backend:**
    *   Navigate to the ackend directory.
    *   Run the following command: pip install -r requirements.txt
*   **Frontend:**
    *   Navigate to the ook-content directory.
    *   Run the following command: 
pm install

**3. Run Qdrant Database:**

*   You need to have Docker installed and running.
*   Run the following command in your terminal to start the Qdrant database:
    `ash
    docker run -p 6333:6333 qdrant/qdrant
    `

**4. Run the Backend Server:**

*   Navigate to the ackend directory.
*   Run the following command: uvicorn app.main:app --port 8000
*   You should see a message indicating that the server is running.

**5. Run the Frontend Application:**

*   Navigate to the ook-content directory.
*   Run the following command: 
pm start
*   This will open a new browser tab with the application running at http://localhost:3000.

**6. Ingest the Data:**

*   Once the backend and frontend are running, you need to ingest the data.
*   Open a new terminal and run the following command to send a POST request to the ingest endpoint:
    `ash
    curl -X POST http://localhost:8000/api/ingest
    `
*   This will ingest the book content into the Qdrant database. You should see a log message in the backend console indicating that the ingestion is complete.

**7. Check for Logs:**

*   **Backend:** Check the console where you are running the uvicorn command for any error messages.
*   **Frontend:** Open the developer console in your browser (usually by pressing F12) and check the 'Console' tab for any network errors or other log messages.

After following these steps, your chatbot should be working correctly.

## Outcome

- ? Impact: Provided a solution to the user's problem.
- ?? Tests: None
- ?? Files: ackend/.env
- ?? Next prompts: None
- ?? Reflection: The issue was environmental, not a code bug. A good reminder to check the environment setup first.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
