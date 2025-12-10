# Physical AI & Humanoid Robotics Textbook with RAG Chatbot

A comprehensive 13-week curriculum for Computer Science and Engineering students learning Physical AI, Humanoid Robotics, ROS 2, and embodied intelligence. This project includes an integrated RAG (Retrieval-Augmented Generation) chatbot that helps students ask questions about the course content.

## Project Overview

This repository contains a comprehensive 13-week Physical AI & Humanoid Robotics textbook website built with Docusaurus v3.x and deployed on GitHub Pages. The implementation includes 26 structured chapters with learning objectives, code examples, exercises, and summaries. The site features responsive design, syntax-highlighted code examples for Python/YAML/XML, Mermaid diagrams for ROS 2 architecture, and an automated GitHub Actions deployment pipeline.

Additionally, the project includes a RAG chatbot with:
- Interactive chat interface embedded in the textbook pages
- Content-based responses using vector search in Qdrant
- Session-based conversation management
- Citations to specific chapters and modules in responses
- Guardrails to ensure responses are limited to textbook content only

## Project Structure

```
physical-ai-and-humanoid-robotics/
├── docs/                    # Docusaurus frontend
│   ├── src/components/ChatWidget/    # Chatbot widget component
│   ├── src/pages/           # Custom pages
│   └── docs/               # Course content
├── backend/                # FastAPI backend
│   ├── api/routes/         # API endpoints
│   ├── models/             # Data models
│   ├── services/           # Business logic
│   ├── utils/              # Utility functions
│   └── tests/              # Backend tests
├── static/                 # Static assets
├── .env.example            # Environment variables template
├── requirements.txt        # Python dependencies
├── package.json            # Frontend dependencies
└── .specify/               # SpecKit Plus files
```

## Local Setup

### Prerequisites

- Node.js 18+ installed
- Python 3.10+ installed
- npm or yarn package manager
- Git for version control
- Access to OpenAI API
- Qdrant Cloud account (free tier)
- Neon Postgres account (free tier)

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/physical-ai-and-humanoid-robotics.git
   cd physical-ai-and-humanoid-robotics
   ```

2. **Backend Setup**:
   ```bash
   # Create and activate virtual environment
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate

   # Install Python dependencies
   pip install --upgrade pip
   pip install -r requirements.txt
   ```

3. **Frontend Setup**:
   ```bash
   # Install Node.js dependencies
   npm install
   ```

4. **Environment Configuration**:
   ```bash
   # Create .env file from example
   cp .env.example .env
   # Update .env with your API keys and configuration
   ```

### Local Development

**Backend (FastAPI)**:
```bash
# Activate virtual environment first
source venv/bin/activate
cd backend
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

The backend API will be available at `http://localhost:8000` with automatic documentation at:
- Interactive API docs: `http://localhost:8000/docs`
- Alternative API docs: `http://localhost:8000/redoc`

**Frontend (Docusaurus)**:
```bash
npm start
```

The frontend will be available at `http://localhost:3000`.

### Build & Production

**Frontend Build**:
```bash
npm run build
```

**Backend Deployment**:
The application can be deployed using Railway with the provided configuration.

## API Documentation

The backend provides a comprehensive REST API with automatic OpenAPI documentation:

### Available Endpoints

#### Health Check
- `GET /api/v1/health` - Basic health check
- `GET /api/v1/health/full` - Full health check including all services

#### Chat Functionality
- `POST /api/v1/chat` - Process chat messages using RAG
  - Request body: `{"message": "your question", "session_id": "unique session", "model": "gpt-4o", "conversation_context": []}`
  - Response: `{"response": "answer", "sources": [...], "conversation_id": "...", "message_id": "..."}`

#### Content Ingestion
- `POST /api/v1/ingest` - Ingest content into vector database
- `POST /api/v1/ingest-batch` - Batch ingestion of multiple content chunks
- `POST /api/v1/ingest-directory` - Ingest all content from a directory
- `POST /api/v1/initialize-db` - Initialize Qdrant collection
- `GET /api/v1/ingestion-stats` - Get ingestion statistics

### API Documentation Access
- Interactive API docs: `http://localhost:8000/docs`
- Alternative API docs: `http://localhost:8000/redoc`
- OpenAPI JSON spec: `http://localhost:8000/openapi.json`

### Example API Usage

**Chat Endpoint**:
```bash
curl -X POST "http://localhost:8000/api/v1/chat" \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is ROS 2?",
    "session_id": "session-123",
    "model": "gpt-4o"
  }'
```

**Ingest Content**:
```bash
curl -X POST "http://localhost:8000/api/v1/ingest" \
  -H "Content-Type: application/json" \
  -d '{
    "module": "ROS 2 Fundamentals",
    "week": "Week 3",
    "chapter_title": "ROS 2 Architecture",
    "content": "ROS 2 is a flexible framework for writing robot software...",
    "url": "/docs/week-03/ros2-architecture"
  }'
```

## Environment Variables

Required environment variables for the backend:

- `OPENAI_API_KEY`: Your OpenAI API key for embeddings and responses
- `QDRANT_URL`: Your Qdrant cluster URL (e.g., https://your-cluster.us-east4-1.gcp.cloud.qdrant.io:6333)
- `QDRANT_API_KEY`: Your Qdrant API key
- `NEON_DATABASE_URL`: Your Neon Postgres connection string
- `MODEL_NAME`: The model to use for responses (default: gpt-4o)
- `EMBEDDING_MODEL`: The model to use for embeddings (default: text-embedding-3-small)

### Environment Configuration

#### Required Services
Before running the application, you need to set up the following services:

1. **OpenAI API Key**
   - Sign up at [OpenAI](https://platform.openai.com/)
   - Create an API key in the dashboard
   - Set `OPENAI_API_KEY` in your environment

2. **Qdrant Vector Database**
   - Sign up at [Qdrant Cloud](https://cloud.qdrant.io/)
   - Create a cluster and note the URL and API key
   - Set `QDRANT_URL` and `QDRANT_API_KEY` in your environment

3. **Neon Postgres Database**
   - Sign up at [Neon](https://neon.tech/)
   - Create a project and get the connection string
   - Set `NEON_DATABASE_URL` in your environment

#### Optional Configuration
- `MODEL_NAME`: OpenAI model to use for responses (default: `gpt-4o`)
- `EMBEDDING_MODEL`: OpenAI model to use for embeddings (default: `text-embedding-3-small`)

#### Setting Up Environment Variables
1. Copy the example file: `cp .env.example .env`
2. Update the values with your actual credentials
3. The application will automatically load these variables when started

## Features

- Interactive RAG chatbot that answers questions about the textbook content
- Content ingestion pipeline to populate the vector database
- Session-based conversation management
- Citations to specific chapters and modules in responses
- Guardrails to ensure responses are limited to textbook content only
- Automatic API documentation with OpenAPI/Swagger
- Comprehensive test coverage
- Cross-platform compatibility

## Deployment

### Frontend (GitHub Pages)
The frontend site is automatically deployed to GitHub Pages via GitHub Actions when changes are pushed to the main branch.

### Backend (Railway)
The backend can be deployed to Railway using the provided configuration files:

1. **Railway Configuration Files**:
   - `backend/Dockerfile` - Container configuration
   - `backend/railway.toml` - Railway-specific deployment settings
   - `backend/Procfile` - Process type definitions

2. **Deployment Steps**:
   - Connect your Railway account to this repository
   - Select the backend directory for deployment
   - Add the required environment variables:
     - `OPENAI_API_KEY`
     - `QDRANT_URL`
     - `QDRANT_API_KEY`
     - `NEON_DATABASE_URL`
     - `MODEL_NAME` (optional, defaults to gpt-4o)
     - `EMBEDDING_MODEL` (optional, defaults to text-embedding-3-small)
   - Deploy the service

3. **Environment Variables**:
   Railway will automatically detect and use the environment variables you set in the dashboard.

4. **CI/CD Pipeline**:
   The repository includes a GitHub Actions workflow for automated testing and deployment:
   - `.github/workflows/backend-ci-cd.yml` - Tests and deploys backend on push to main
   - The workflow runs tests, builds a Docker image, and deploys to Railway
   - Required secrets: `RAILWAY_TOKEN` and `RAILWAY_PROJECT_ID`

### Local Production
To run in production-like mode locally:
```bash
# Backend
cd backend
gunicorn main:app -w 4 -k uvicorn.workers.UvicornWorker --bind 0.0.0.0:8000

# Or with uvicorn
uvicorn main:app --host 0.0.0.0 --port 8000 --workers 4
```

## Troubleshooting

### Common Issues

1. **Port already in use**: Make sure ports 3000 (frontend) and 8000 (backend) are available.
   - Solution: Use `lsof -i :8000` or `lsof -i :3000` to identify processes using the ports, then kill them with `kill -9 <PID>` or change the port in the configuration.

2. **Missing environment variables**: Ensure all required environment variables are set in your `.env` file.
   - Solution: Copy `.env.example` to `.env` and fill in your actual credentials.

3. **Database connection errors**: Verify your Neon Postgres connection string is correct.
   - Solution: Check the format: `postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require`
   - Ensure the database is active in Neon dashboard
   - Check if your IP is whitelisted (if applicable)

4. **Qdrant connection errors**: Check your Qdrant URL and API key.
   - Solution: Verify the URL format: `https://your-cluster.region.gcp.cloud.qdrant.io:6333`
   - Ensure the API key is correct and has necessary permissions
   - Test connection with curl: `curl -X GET "YOUR_QDRANT_URL/collections" -H "api-key: YOUR_API_KEY"`

5. **OpenAI API errors**: Verify your OpenAI API key is valid and has sufficient credits.
   - Solution: Check your OpenAI dashboard for API key validity and billing status
   - Ensure the account has access to the specified models (gpt-4o, text-embedding-3-small)

6. **Module import errors**: If you see errors like `ModuleNotFoundError` or `ImportError`.
   - Solution: Ensure you're running in the correct virtual environment and dependencies are installed with `pip install -r requirements.txt`

7. **Qdrant collection not found**: If the application fails to find the Qdrant collection.
   - Solution: Run the ingestion pipeline to create the collection and populate it with content

8. **CORS errors**: If the frontend can't communicate with the backend.
   - Solution: Check the CORS configuration in `main.py` and ensure the frontend URL is included in the allowed origins

### Performance Issues

1. **Slow response times**:
   - Check if OpenAI API is responding slowly (common during high usage)
   - Verify database connection performance
   - Ensure Qdrant cluster has sufficient resources

2. **High memory usage**:
   - Monitor memory usage during ingestion operations
   - Consider batching large content uploads
   - Check for memory leaks in long-running processes

### Debugging Tips

1. **Enable detailed logging**: Set environment variable `LOG_LEVEL=DEBUG` to get more detailed logs.

2. **Check API documentation**: Use the built-in API docs at `http://localhost:8000/docs` to test endpoints directly.

3. **Database debugging**: Use database tools to inspect conversation and message tables if needed.

4. **Vector database debugging**: Use Qdrant dashboard to inspect collections and vector points.

### API Testing
You can test the API endpoints using the built-in Swagger UI at `http://localhost:8000/docs` or with command-line tools like curl.

### Running Tests
```bash
# Backend tests
cd backend
python -m pytest tests/ -v

# Get test coverage
python -m pytest tests/ --cov=backend --cov-report=html
```

## Development

### Backend Structure
- `api/routes/` - API endpoints and request/response models
- `models/` - Database models and schemas
- `services/` - Business logic and core functionality
- `utils/` - Helper functions and utilities
- `tests/` - Unit and integration tests

### Frontend Structure
- `src/components/` - React components including the chat widget
- `docs/` - Course content and textbook materials
- `src/pages/` - Custom pages and layouts

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Add tests if applicable
5. Run tests to ensure everything works (`python -m pytest` in backend)
6. Commit your changes (`git commit -m 'Add amazing feature'`)
7. Push to the branch (`git push origin feature/amazing-feature`)
8. Open a Pull Request
