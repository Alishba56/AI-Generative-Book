import os
import google.generativeai as genai
from dotenv import load_dotenv

load_dotenv()

class GeminiService:
    def __init__(self):
        self.api_key = os.environ.get("GEMINI_API_KEY")
        if not self.api_key:
            raise ValueError("GEMINI_API_KEY environment variable not set.")
        genai.configure(api_key=self.api_key)
        self.model = genai.GenerativeModel('gemini-pro')

    def generate_text(self, prompt: str) -> str:
        """
        Generates text using the Gemini Pro model.
        """
        try:
            print(f"Sending prompt to Gemini: {prompt}")
            response = self.model.generate_content(prompt)
            print(f"Received response from Gemini: {response.text}")
            return response.text
        except Exception as e:
            # Handle potential exceptions from the API call
            print(f"Error generating text with Gemini: {type(e)}")
            print(f"Error details: {e}")
            return "Sorry, I encountered an error while processing your request."

    def generate_text_stream(self, prompt: str):
        """
        Generates text using the Gemini Pro model and streams the response.
        """
        try:
            print(f"Sending streaming prompt to Gemini: {prompt}")
            response_stream = self.model.generate_content(prompt, stream=True)
            for chunk in response_stream:
                yield chunk.text
        except Exception as e:
            print(f"Error streaming text with Gemini: {e}")
            yield "Sorry, I encountered an error while processing your request."

gemini_service = GeminiService()
