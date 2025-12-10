from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

# Import routes and error handlers
from backend.api.routes.health import router as health_router
from backend.api.routes.chat import router as chat_router
from backend.api.routes.ingest import router as ingest_router
from backend.utils.error_handlers import register_error_handlers

app = FastAPI(
    title="RAG Chatbot API for Physical AI & Humanoid Robotics Textbook",
    description="API for the RAG chatbot that answers questions about Physical AI and Humanoid Robotics curriculum",
    version="1.0.0"
)

# Configure CORS for GitHub Pages domain
# Allow the deployed GitHub Pages site and local development servers
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://sheikh-ubaid-raza.github.io",  # Deployed GitHub Pages
        "http://localhost:3000",               # Local Docusaurus development
        "http://localhost:3001",               # Alternative local Docusaurus port
        "http://127.0.0.1:3000",               # Alternative localhost format
        "http://127.0.0.1:3001"                # Alternative localhost format
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Expose headers for client access
    expose_headers=["Access-Control-Allow-Origin"]
)

# Include routes
app.include_router(health_router, prefix="/api/v1", tags=["health"])
app.include_router(chat_router, prefix="/api/v1", tags=["chat"])
app.include_router(ingest_router, prefix="/api/v1", tags=["ingest"])

# Initialize error handlers
register_error_handlers(app)

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API for Physical AI & Humanoid Robotics Textbook"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)