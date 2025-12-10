"""
Script to generate OpenAPI specification for the RAG Chatbot API
This script will generate a JSON file containing the OpenAPI specification
"""
import json
from backend.main import app

def generate_openapi_spec():
    """
    Generate OpenAPI specification and save to file
    """
    openapi_spec = app.openapi()

    # Save to file
    with open("openapi.json", "w") as f:
        json.dump(openapi_spec, f, indent=2)

    print("OpenAPI specification generated successfully!")
    print(f"Available endpoints: {len(openapi_spec['paths'])}")
    print("Available tags:", openapi_spec.get('tags', []))

    return openapi_spec

if __name__ == "__main__":
    spec = generate_openapi_spec()