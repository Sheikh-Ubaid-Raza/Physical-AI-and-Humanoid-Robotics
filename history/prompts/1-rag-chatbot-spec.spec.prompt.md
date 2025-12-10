---
id: 1
title: rag-chatbot-spec
stage: spec
date: '2025-12-08'
surface: agent
model: claude-sonnet-4-5-20250929
feature: rag-chatbot
branch: '1-rag-chatbot'
user: Claude
command: /sp.specify
labels:
  - feature
  - specification
  - rag
  - chatbot
  - education
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files: |
  - specs/1-rag-chatbot/spec.md
  - specs/1-rag-chatbot/checklists/requirements.md
tests: |
  -
outcome: Specification created for RAG Chatbot feature
evaluation: Specification meets all quality criteria and is ready for planning
---

# Prompt

```
RAG Chatbot for Physical AI & Humanoid Robotics Textbook - Part 1 (Core Features)

Target audience: Students learning Physical AI through interactive Docusaurus textbook
Focus: Basic RAG chatbot that answers questions about book content using vector search
Success criteria:
- User can ask questions in chat widget and receive accurate answers from book content
- Chatbot retrieves relevant content from Qdrant vector database
- Responses include citations to specific chapters/modules
- Chat history persists across page navigation (same session)
- System deploys successfully: Frontend on GitHub Pages, Backend on Railway
- Answers 20 test queries accurately with book content only

Constraints:
- Free tier services only: OpenAI, Qdrant Cloud (1GB), Neon Postgres, Railway ($5 credit)
- Tech stack: Docusaurus + React (frontend), FastAPI + Python 3.10 (backend)
- Must use: OpenAI Agent SDK, gemini-embeddings-model, Qdrant, Neon Postgres
- Project structure: Separate /docs and /backend directories
- Embeddings: 500-1000 token chunks, 100-token overlap, top-5 retrieval

Core features to build:
1. Document ingestion pipeline: Parse Docusaurus markdown → chunk → embed → store in Qdrant
2. FastAPI backend with endpoints:
   - POST /api/v1/chat (main chat endpoint)
   - GET /api/v1/health (health check)
   - POST /api/v1/ingest (document ingestion)
3. RAG pipeline: Query → embed → Qdrant search → OpenAI Agent → response
4. Chat history storage in Neon Postgres (conversations, messages tables)
5. React chat widget component embedded in Docusaurus (bottom-right corner)
6. Guardrails: System prompt restricts responses to book content only
7. Session management: Generate session_id per user, maintain context
8. CORS configuration: Allow GitHub Pages → Railway communication
9. Response streaming: Real-time typing effect in chat widget
10. Deployment: GitHub Pages (static site), Railway (backend API)

Not building in Part 1:
- Selected text feature (user highlights text → ask AI) - deferred to Part 2
- Better-auth signup/signin - bonus feature for Part 2
- Content personalization based on user background - bonus for Part 2
- Urdu translation feature - bonus for Part 2
- Claude Code Subagents/Skills - will be used during development but not a deliverable feature
- Advanced analytics dashboard
- Response caching or optimization
- Multi-language support beyond English

Technical requirements:
- Backend: Python 3.10+, FastAPI with async/await, Pydantic models, pytest (70% coverage)
- Frontend: Docusaurus v3.x, React components, TypeScript/JavaScript
- Database schemas:
  - Neon: conversations(id, session_id, created_at), messages(id, conversation_id, role, content, sources, timestamp)
  - Qdrant: collection "physical_ai_book", 1536-dim vectors, payload={module, week, chapter_title, content, url}
- Environment variables: OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL
- Error handling: Graceful degradation with user-friendly messages
- Logging: All API requests/responses logged for debugging

Deliverables:
1. Functional chat widget on deployed GitHub Pages site
2. Working FastAPI backend on Railway
3. Qdrant collection populated with book content embeddings
4. Neon database with chat history tables
5. README files with setup instructions (root, backend, frontend)
6. .env.example files for both frontend and backend
7. Test suite with 70% backend coverage


Success validation:
- User visits book → opens chatbot → asks "What is ROS 2?" → receives answer citing Module 1
- Chatbot refuses off-topic questions like "What's the weather?"
- Chat history maintained: User asks follow-up question, context preserved
- All endpoints respond within 5 seconds
- No console errors on frontend
- Backend tests pass with 70% coverage
```

# Response

A comprehensive specification has been created for the RAG Chatbot feature. The specification includes:

- Feature overview with business value
- User scenarios and edge cases
- Functional requirements for all aspects of the chatbot
- Non-functional requirements for performance, security, and reliability
- Key entities for data modeling
- Success criteria with both quantitative and qualitative measures
- Assumptions, dependencies, and constraints

The specification is ready for the planning phase and meets all quality criteria.