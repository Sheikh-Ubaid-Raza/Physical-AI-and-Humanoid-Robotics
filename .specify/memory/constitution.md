<!-- Sync Impact Report:
Version change: 0.0.0 → 1.0.0
Modified principles: None (initial creation)
Added sections: Code Quality, Documentation, Architecture, Security, Testing, Version Control, Performance, Content Standards, RAG Chatbot, Constraints, Success Criteria
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ✅ updated
  - .specify/templates/spec-template.md: ✅ updated
  - .specify/templates/tasks-template.md: ✅ updated
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Interactive Textbook Constitution

## Core Principles

### Educational Clarity
Educational clarity for students with varying backgrounds. The content must be accessible and understandable to individuals with diverse levels of prior knowledge in AI, robotics, and programming.

### Technical Accuracy
Technical accuracy in all code examples and configurations. All provided code snippets, configurations, and theoretical explanations must be precise, verifiable, and free of errors.

### AI-Native Design
AI-native design with integrated RAG chatbot. The project will leverage AI capabilities, including a Retrieval-Augmented Generation (RAG) chatbot, to enhance the learning experience and provide interactive assistance.

### Modular and Maintainable Architecture
The system will be built with a modular and maintainable architecture. Components should be loosely coupled, independently deployable where appropriate, and easy to understand, modify, and extend.

## Key Standards

### Code Quality
- Python 3.10+, TypeScript for frontend.
- Type hints mandatory, Google-style docstrings.
- Black/Prettier formatting, strict linting.
- Max function: 50 lines, max file: 500 lines.

### Documentation
- Each chapter: learning objectives, prerequisites, exercises, commented code examples.
- Step-by-step installation with verification commands.
- README files for root, backend, frontend, docs.

### Architecture
- Frontend: Docusaurus v3.x + React components.
- Backend: FastAPI (async/await).
- Database: Neon Serverless Postgres.
- Vector Store: Qdrant Cloud (free tier).
- Deployment: GitHub Pages + Railway/Render.

### Security
- API keys in environment variables only (.env never committed).
- Input validation on all endpoints.
- Parameterized queries, JWT with 24h expiry.
- Rate limiting: 100 requests/min per user.

### Testing
- 70% backend unit test coverage (pytest).
- Integration tests for RAG pipeline.
- CI/CD via GitHub Actions.

### Version Control
- Feature branches, PRs mandatory.
- Conventional Commits (feat:, fix:, docs:).
- No direct commits to main.

### Performance
- Page load: <3s on 4G.
- Chatbot response: <5s.
- Lazy loading for images/heavy components.

### Content Standards
- Structure: Intro → Theory → Example → Exercises → Summary.
- Copy-paste executable code with setup instructions.
- Balance: 60% code, 40% explanation.
- Hardware requirements clearly stated per module.

### RAG Chatbot
- Embeddings: OpenAI text-embedding-3-small.
- Chunk size: 500-1000 tokens, 100-token overlap.
- Top-5 retrieval, confidence threshold: 0.6.
- Support selected text queries.
- Include chapter references in responses.

## Constraints

- Deadline: Nov 30, 2025, 6:00 PM.
- Demo video: Max 90 seconds.
- Free tier services only.
- No hardcoded secrets in repo.

## Success Criteria

- Deploys successfully to GitHub Pages.
- Covers all 13 weeks of course content.
- RAG chatbot answers accurately (20 test queries).
- Selected text feature functional.
- Runs error-free on fresh Ubuntu 22.04.

## Governance

All PRs/reviews MUST verify compliance with the principles and standards outlined in this constitution. Complexity MUST be justified through a clear rationale and adherence to modular design. The project MUST adhere to the constraint of using only free-tier services. No hardcoded secrets are permitted in the repository; all sensitive information MUST be managed via environment variables. The project MUST meet the deadline of November 30, 2025, 6:00 PM. The demo video MUST be a maximum of 90 seconds. The project MUST deploy successfully to GitHub Pages. The content MUST cover all 13 weeks of the course content. The RAG chatbot MUST answer accurately for 20 provided test queries. The selected text feature MUST be functional. The entire system MUST run error-free on a fresh Ubuntu 22.04 installation.

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06
