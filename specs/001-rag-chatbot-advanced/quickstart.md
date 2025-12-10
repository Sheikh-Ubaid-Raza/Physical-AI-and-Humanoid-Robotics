# Quickstart: RAG Chatbot Advanced Features

## Prerequisites

- Python 3.10+
- Node.js 18+ (for Docusaurus frontend)
- Google Gemini API key
- Qdrant Cloud account (free tier)
- Neon Postgres account (free tier)
- Git

## Environment Setup

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd physical-ai-and-humanoid-robotics
   ```

2. **Create Python virtual environment**:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install backend dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

4. **Set up environment variables**:
   ```bash
   cp .env.example .env
   ```

   Edit `.env` with your credentials:
   ```env
   GEMINI_API_KEY=your_gemini_api_key_here
   QDRANT_URL=https://your-cluster-id.xxxx.aws.cloud.qdrant.io:6333
   QDRANT_API_KEY=your_qdrant_api_key_here
   NEON_DATABASE_URL=postgresql://user:password@project.neon.tech/dbname?sslmode=require
   MODEL_NAME=gemini-2.5-flash
   EMBEDDING_MODEL=embedding-001
   JWT_SECRET_KEY=your_random_32_char_secret_here
   JWT_ALGORITHM=HS256
   JWT_EXPIRY_HOURS=24
   ```

## Backend Setup

1. **Initialize the database**:
   ```bash
   python -c "from backend.utils.init_db import init_db; init_db()"
   ```

2. **Update Qdrant collection** (ensure 768 dimensions for Gemini embeddings):
   ```bash
   # This will recreate the collection with the correct dimensions
   python -c "from backend.utils.qdrant_setup import recreate_collection; recreate_collection()"
   ```

3. **Run the backend server**:
   ```bash
   uvicorn backend.main:app --host 0.0.0.0 --port 8000 --reload
   ```

## Frontend Setup

1. **Install frontend dependencies**:
   ```bash
   npm install
   ```

2. **Start the Docusaurus development server**:
   ```bash
   npm start
   ```

## API Endpoints

### Authentication
- `POST /api/v1/auth/signup` - User registration
- `POST /api/v1/auth/signin` - User login
- `POST /api/v1/auth/signout` - User logout
- `GET /api/v1/auth/me` - Get current user

### Chat & Selection
- `POST /api/v1/chat` - Standard chat endpoint
- `POST /api/v1/chat/selection` - Selected text feature
- `GET /api/v1/health` - Health check

### Personalization & Translation
- `POST /api/v1/personalize` - Content personalization (requires auth)
- `POST /api/v1/translate` - Urdu translation (requires auth)

### Content Ingestion
- `POST /api/v1/ingest` - Ingest content into vector database

## Key Features Setup

### 1. Selected Text Feature
- Text selection detection works automatically on all textbook pages
- Floating tooltip appears when text is highlighted
- Selected text is sent to the `/api/v1/chat/selection` endpoint

### 2. User Authentication
- Users can sign up with background information (software/hardware experience)
- JWT tokens are issued with 24-hour expiration
- Protected routes require valid JWT in Authorization header

### 3. Content Personalization
- Requires authentication
- "Personalize Content" button appears on each chapter for logged-in users
- Adjusts content based on user's background level (beginner/intermediate/advanced)

### 4. Urdu Translation
- Requires authentication
- "Translate to Urdu" button appears on each chapter for logged-in users
- Preserves code blocks and technical terms in English

## Testing

1. **Run backend tests**:
   ```bash
   python -m pytest backend/tests/ -v
   ```

2. **Run frontend tests**:
   ```bash
   npm test
   ```

## Configuration Notes

- The system uses Google Gemini (gemini-2.5-flash) instead of OpenAI
- Embeddings are 768-dimensional (Gemini embedding-001) instead of 1536-dimensional
- All new features build on existing Part 1 functionality without breaking changes
- Response time target: <5 seconds for all features
- JWT tokens expire after 24 hours

## Deployment

### Backend (Railway/Render)
```bash
# For Railway
railway up

# For Render
# Push to GitHub and configure auto-deploy
```

### Frontend (GitHub Pages)
```bash
npm run build
# Deploy the build folder to GitHub Pages
```

## Troubleshooting

1. **Qdrant dimension error**: Ensure your Qdrant collection is set up with 768 dimensions for Gemini embeddings
2. **JWT authentication fails**: Check that JWT_SECRET_KEY is properly set in environment
3. **Gemini API errors**: Verify GEMINI_API_KEY is correct and has sufficient quota
4. **Selected text not working**: Check browser console for JavaScript errors related to text selection