import os
import time
from PIL import Image
from google import genai

# 1. Initialize the client and load the image
client = genai.Client(api_key="PASTE KEY")
image_path = "deer.jpg"
img = Image.open(image_path)

max_retries = 5
retry_delay = 2 # Start by waiting 2 seconds

# 2. Loop to retry the API call if the server is busy
for attempt in range(max_retries):
    try:
        response_stream = client.models.generate_content_stream(
            model="gemini-3.8-flash",
            contents=[img, "Is this a deer? Answer it yes or no."]
        )
        
        # Print the streaming response
        for chunk in response_stream:
            print(chunk.text, end="", flush=True)
        
        print() # Final newline
        break # Success! Exit the retry loop
        
    except Exception as e:
        # Check if the error is due to the server being unavailable (503)
        if "503" in str(e):
            print(f"\nServer busy (Attempt {attempt + 1}/{max_retries}). Retrying in {retry_delay} seconds...")
            time.sleep(retry_delay)
            retry_delay *= 2 # Double the wait time for the next attempt (exponential backoff)
        else:
            # If it's a different kind of error (like file not found), stop and raise it
            print(f"\nAn unexpected error occurred: {e}")
            break
            