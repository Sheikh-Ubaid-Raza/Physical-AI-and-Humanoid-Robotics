# Implementation Plan: RAG Chatbot Advanced Features

**Branch**: `001-rag-chatbot-advanced` | **Date**: 2025-12-10 | **Spec**: [link](/mnt/c/hackathons/hackathon-1/Physical-AI-and-Humanoid-Robotics/specs/001-rag-chatbot-advanced/spec.md)
**Input**: Feature specification from `/specs/001-rag-chatbot-advanced/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of advanced RAG chatbot features for the Physical AI & Humanoid Robotics textbook, including selected text queries, user authentication with JWT, content personalization based on user background, and Urdu translation. The system will use Google Gemini (gemini-2.5-flash) for all LLM operations instead of OpenAI, with 768-dimensional embeddings to align with Gemini embedding-001 model. The architecture builds on the existing Part 1 implementation without breaking changes, adding protected routes for premium features and maintaining response times under 5 seconds.

## Technical Context

**Language/Version**: Python 3.10+ for backend, TypeScript/JavaScript for frontend
**Primary Dependencies**: FastAPI, Docusaurus v3.x, React, python-jose, passlib[bcrypt], google-generativeai, OpenAI Agents SDK (configured for Gemini)
**Storage**: Neon Serverless Postgres (users, personalized_content, translations, conversations, messages), Qdrant Cloud (768-dim embeddings)
**Testing**: pytest for backend, Jest for frontend
**Target Platform**: Web application (Linux server backend, Docusaurus frontend)
**Project Type**: web (determines source structure)
**Performance Goals**: <5s response time for all features, handle 100 concurrent users
**Constraints**: Free tier services only (Gemini, Qdrant 1GB, Neon 3GB), <5s response time for all features, mobile-responsive UI
**Scale/Scope**: 1000+ students, 13-week curriculum content, 50+ chapters

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Educational Clarity**: All documentation and code examples MUST prioritize educational clarity for students with varying backgrounds.
- **Technical Accuracy**: All code examples and configurations MUST be technically accurate and verifiable.
- **AI-Native Design**: Design MUST integrate AI capabilities, specifically a RAG chatbot, to enhance learning.
- **Modular and Maintainable Architecture**: The architecture MUST be modular, loosely coupled, and maintainable.
- **Code Quality**: Adherence to Python 3.10+, TypeScript, type hints, Google-style docstrings, Black/Prettier formatting, strict linting, max 50 lines per function, and max 500 lines per file is MANDATORY.
- **Documentation**: Each chapter MUST include learning objectives, prerequisites, exercises, and commented code examples. Step-by-step installation with verification commands and README files for root, backend, frontend, docs are REQUIRED.
- **Architecture**: Frontend MUST use Docusaurus v3.x + React components; Backend MUST use FastAPI (async/await); Database MUST be Neon Serverless Postgres; Vector Store MUST be Qdrant Cloud (free tier); Deployment MUST target GitHub Pages + Railway/Render.
- **Security**: API keys MUST only be in environment variables (.env never committed). Input validation on all endpoints, parameterized queries, JWT with 24h expiry, and rate limiting (100 requests/min per user) are REQUIRED.
- **Testing**: 70% backend unit test coverage (pytest), integration tests for RAG pipeline, and CI/CD via GitHub Actions are REQUIRED.
- **Version Control**: Feature branches, PRs, Conventional Commits (feat:, fix:, docs:), and no direct commits to main are MANDATORY.
- **Performance**: Page load MUST be <3s on 4G; Chatbot response MUST be <5s; Lazy loading for images/heavy components is REQUIRED.
- **Content Standards**: Structure MUST be Intro → Theory → Example → Exercises → Summary. Copy-paste executable code with setup instructions, balance of 60% code and 40% explanation, and clear hardware requirements per module are REQUIRED.
- **RAG Chatbot**: Embeddings MUST use Google Gemini embedding-001 (768 dimensions) instead of OpenAI. Chunk size: 500-1000 tokens with 100-token overlap. Top-5 retrieval, confidence threshold: 0.6. Support for selected text queries and inclusion of chapter references in responses are REQUIRED.

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot-advanced/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── api/
│   ├── routes/
│   │   ├── auth.py          # Authentication endpoints
│   │   ├── chat.py          # Chat and selection endpoints
│   │   ├── health.py        # Health check endpoint
│   │   └── ingest.py        # Content ingestion endpoint
│   └── __init__.py
├── models/
│   ├── user.py              # User model with background information
│   ├── personalized_content.py # Personalized content model
│   ├── translation.py       # Translation model
│   ├── conversation.py      # Conversation model (from Part 1)
│   └── message.py           # Message model (from Part 1)
├── services/
│   ├── auth_service.py      # Authentication and JWT handling
│   ├── chat_service.py      # Chat functionality (enhanced from Part 1)
│   ├── ingestion_service.py # Content ingestion (from Part 1)
│   ├── rag_service.py       # RAG functionality (enhanced from Part 1)
│   ├── personalization_service.py # Content personalization
│   └── translation_service.py # Urdu translation
├── utils/
│   ├── database.py          # Database utilities
│   ├── embedding.py         # Embedding utilities (updated for Gemini)
│   ├── qdrant_setup.py      # Qdrant setup (updated for 768-dim)
│   ├── qdrant_client.py     # Qdrant client
│   ├── security.py          # Security utilities (JWT, password hashing)
│   ├── content_parser.py    # Content parsing utilities
│   ├── chunker.py           # Content chunking utilities
│   ├── validation.py        # Validation utilities
│   ├── rate_limiter.py      # Rate limiting utilities
│   ├── logging.py           # Logging utilities
│   ├── performance_monitor.py # Performance monitoring
│   ├── cache.py             # Caching utilities
│   ├── timeout_handler.py   # Timeout handling
│   ├── performance_dashboard.py # Performance dashboard
│   └── init_db.py           # Database initialization
├── tests/
│   ├── test_auth.py         # Authentication tests
│   ├── test_personalization.py # Personalization tests
│   ├── test_translation.py  # Translation tests
│   ├── test_chat.py         # Chat functionality tests
│   ├── test_api.py          # API endpoint tests
│   ├── test_rag_service.py  # RAG service tests
│   ├── test_ingestion.py    # Ingestion tests
│   ├── test_models.py       # Model tests
│   ├── test_e2e_stories.py  # End-to-end test stories
│   ├── test_chat_service_unit.py # Chat service unit tests
│   ├── test_chat_service_methods.py # Chat service method tests
│   └── conftest.py          # Test configuration
├── main.py                  # Application entry point
├── generate_openapi.py      # OpenAPI generation
└── Dockerfile               # Containerization

src/
├── components/
│   ├── ChatWidget/
│   │   ├── ChatWidget.tsx   # Main chat widget component
│   │   ├── ChatModal.tsx    # Chat modal component
│   │   ├── MessageBubble.tsx # Message bubble component
│   │   ├── api.ts           # API service (enhanced for auth/selection)
│   │   └── __tests__/
│   │       └── jest.setup.ts # Jest setup for testing
│   └── HomepageFeatures/    # Homepage features
│       └── index.tsx
├── theme/
│   └── Root.tsx             # Theme root component
└── pages/
    └── index.tsx            # Homepage

docs/
├── src/
│   └── css/
│       └── custom.css       # Custom CSS
├── static/
│   └── img/                 # Static images
├── package.json
├── docusaurus.config.ts     # Docusaurus configuration
├── sidebars.ts              # Sidebar configuration
└── tsconfig.json            # TypeScript configuration

.history/
├── prompts/                 # Prompt history records
└── adr/                     # Architecture decision records

.specify/
├── memory/
│   └── constitution.md      # Project constitution
├── templates/               # Template files
└── scripts/                 # Scripts
```

**Structure Decision**: Web application with separate backend and frontend components. The backend uses FastAPI for API endpoints and services, while the frontend uses Docusaurus with React components. This structure allows for clear separation of concerns while maintaining the existing Part 1 architecture and adding the new advanced features.

### 6. API Contracts

All API contracts have been defined as OpenAPI 3.0 specifications in the contracts directory:

1. **Authentication API** (`auth-api.yaml`):
   - `/auth/signup` - User registration with background information
   - `/auth/signin` - User login with JWT token issuance
   - `/auth/signout` - User logout
   - `/auth/me` - Get current user information

2. **Text Selection API** (`selection-api.yaml`):
   - `/chat/selection` - Process selected text as high-priority query to AI

3. **Content Personalization API** (`personalization-api.yaml`):
   - `/personalize` - Adjust chapter content based on user's background level

4. **Content Translation API** (`translation-api.yaml`):
   - `/translate` - Translate chapter content to Urdu while preserving technical terms

5. **Content Ingestion API** (`ingestion-api.yaml`):
   - `/ingest` - Add textbook content to vector database for RAG retrieval

All APIs use JWT Bearer authentication (except public endpoints which are optional).

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Switch from OpenAI to Gemini embeddings | Compliance with requirement to use only free tier services and avoid OpenAI API | Continuing with OpenAI would violate the core constraint of the project |
| 768-dim instead of 1536-dim embeddings | Gemini embedding-001 model requirement | Would require significant changes to Qdrant collection and break compatibility |
