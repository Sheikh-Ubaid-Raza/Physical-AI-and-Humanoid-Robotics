<!-- Sync Impact Report:
Version change: 1.0.0 → 1.1.0
Modified principles: Simplified and focused on essentials
Added sections: Project Structure
Removed sections: Verbose explanations
Templates requiring updates:
  - .specify/templates/plan-template.md: ✅ updated
  - .specify/templates/spec-template.md: ✅ updated
  - .specify/templates/tasks-template.md: ✅ updated
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Interactive Textbook Constitution

## Core Principles

### Educational Clarity
Content accessible to students with varying AI/robotics/programming backgrounds.

### Technical Accuracy
All code examples, configurations, and explanations must be precise and error-free.

### AI-Native Design
Integrated RAG chatbot using OpenAI Agent SDK for interactive learning assistance.

### Modular Architecture
Loosely coupled components, independently deployable, easy to maintain and extend.

## Key Standards

### Code Quality
- Python 3.10+, TypeScript/JavaScript for frontend
- Type hints mandatory, Google-style docstrings
- Black/Prettier formatting, strict linting
- Max function: 50 lines, max file: 500 lines

### Project Structure
```
physical-ai-and-humanoid-robotics/
├── docs/                    # Docusaurus frontend
│   ├── src/components/ChatWidget/
│   └── docs/               # Course content
├── backend/                # FastAPI backend
│   ├── api/routes/
│   ├── services/
│   ├── utils/
│   └── tests/
└── .specify/               # SpecKit Plus files
```

### Architecture
- Frontend: Docusaurus v3.x + React
- Backend: FastAPI (async/await)
- Database: Neon Serverless Postgres (chat history)
- Vector Store: Qdrant Cloud (embeddings)
- AI: OpenAI Agent SDK + text-embedding-3-small
- Deployment: GitHub Pages (frontend) + Railway (backend)

### Documentation
- README files for root, backend, frontend
- Inline comments for complex logic
- API docs auto-generated via FastAPI Swagger

### Security
- API keys in .env only (never commit .env)
- Input validation with Pydantic
- Rate limiting: 100 requests/min per session

### Testing
- 70% backend test coverage (pytest)
- Integration tests for RAG pipeline
- CI/CD via GitHub Actions

### Performance
- Page load: <3s on 4G
- Chatbot response: <5s
- Response streaming for real-time chat

### RAG Chatbot
- Embeddings: OpenAI text-embedding-3-small (1536 dimensions)
- Chunk size: 500-1000 tokens, 100-token overlap
- Top-5 retrieval, confidence threshold: 0.6
- Guardrails: Book content only, no off-topic responses
- Include chapter references with URLs in responses

## Constraints

- Free tier services only
- No hardcoded secrets in repo
- Must run on Ubuntu 22.04

## Success Criteria

### Core (100 points)
- Deploys to GitHub Pages successfully
- Covers all 13 weeks of course content
- RAG chatbot answers 20 test queries accurately
- Selected text feature functional

### Bonus (up to 150 points)
- Claude Code Subagents/Skills (+50)
- Better-auth signup/signin (+50)
- Content personalization (+50)
- Urdu translation (+50)

## Governance

All code must comply with standards above. Use feature branches and conventional commits (feat:, fix:, docs:). No direct commits to main. Backend and frontend completely separate. All secrets in environment variables.

**Version**: 1.1.0 | **Ratified**: 2025-12-08 | **Last Amended**: 2025-12-08