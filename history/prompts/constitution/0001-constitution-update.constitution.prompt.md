---
id: 0001
title: Constitution Update
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution Project: Physical AI & Humanoid Robotics Interactive Textbook Core Principles: - Educational clarity for students with varying backgrounds Technical accuracy in all code examples and configurations AI-native design with integrated RAG chatbot Modular and maintainable architecture Key Standards: Code Quality: - Python 3.10+, TypeScript for frontend - Type hints mandatory, Google-style docstrings - Black/Prettier formatting, strict linting - Max function: 50 lines, max file: 500 lines Documentation: - Each chapter: learning objectives, prerequisites, exercises, commented code examples - Step-by-step installation with verification commands - README files for root, backend, frontend, docs Architecture: - Frontend: Docusaurus v3.x + React components - Backend: FastAPI (async/await) - Database: Neon Serverless Postgres - Vector Store: Qdrant Cloud (free tier) - Deployment: GitHub Pages + Railway/Render Security: - API keys in environment variables only (.env never committed) - Input validation on all endpoints - Parameterized queries, JWT with 24h expiry - Rate limiting: 100 requests/min per user Testing: - 70% backend unit test coverage (pytest) - Integration tests for RAG pipeline - CI/CD via GitHub Actions Version Control: - Feature branches, PRs mandatory - Conventional Commits (feat:, fix:, docs:) - No direct commits to main Performance: - Page load: <3s on 4G - Chatbot response: <5s - Lazy loading for images/heavy components Content Standards: - Structure: Intro → Theory → Example → Exercises → Summary - Copy-paste executable code with setup instructions - Balance: 60% code, 40% explanation - Hardware requirements clearly stated per module RAG Chatbot: - Embeddings: OpenAI text-embedding-3-small - Chunk size: 500-1000 tokens, 100-token overlap - Top-5 retrieval, confidence threshold: 0.6 - Support selected text queries - Include chapter references in responses Constraints: - Deadline: Nov 30, 2025, 6:00 PM - Demo video: Max 90 seconds - Free tier services only - No hardcoded secrets in repo Success Criteria: - Deploys successfully to GitHub Pages - Covers all 13 weeks of course content - RAG chatbot answers accurately (20 test queries) - Selected text feature functional - Runs error-free on fresh Ubuntu 22.04
labels: ["constitution", "initial-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
  - .specify/templates/spec-template.md
  - .specify/templates/tasks-template.md
tests: []
---

## Prompt

```text
Project: Physical AI & Humanoid Robotics Interactive Textbook

Core Principles:
- Educational clarity for students with varying backgrounds
- Technical accuracy in all code examples and configurations
- AI-native design with integrated RAG chatbot
- Modular and maintainable architecture

Key Standards:

Code Quality:
- Python 3.10+, TypeScript for frontend
- Type hints mandatory, Google-style docstrings
- Black/Prettier formatting, strict linting
- Max function: 50 lines, max file: 500 lines

Documentation:
- Each chapter: learning objectives, prerequisites, exercises, commented code examples
- Step-by-step installation with verification commands
- README files for root, backend, frontend, docs

Architecture:
- Frontend: Docusaurus v3.x + React components
- Backend: FastAPI (async/await)
- Database: Neon Serverless Postgres
- Vector Store: Qdrant Cloud (free tier)
- Deployment: GitHub Pages + Railway/Render

Security:
- API keys in environment variables only (.env never committed)
- Input validation on all endpoints
- Parameterized queries, JWT with 24h expiry
- Rate limiting: 100 requests/min per user

Testing:
- 70% backend unit test coverage (pytest)
- Integration tests for RAG pipeline
- CI/CD via GitHub Actions

Version Control:
- Feature branches, PRs mandatory
- Conventional Commits (feat:, fix:, docs:)
- No direct commits to main

Performance:
- Page load: <3s on 4G
- Chatbot response: <5s
- Lazy loading for images/heavy components

Content Standards:
- Structure: Intro → Theory → Example → Exercises → Summary
- Copy-paste executable code with setup instructions
- Balance: 60% code, 40% explanation
- Hardware requirements clearly stated per module

RAG Chatbot:
- Embeddings: OpenAI text-embedding-3-small
- Chunk size: 500-1000 tokens, 100-token overlap
- Top-5 retrieval, confidence threshold: 0.6
- Support selected text queries
- Include chapter references in responses

Constraints:
- Deadline: Nov 30, 2025, 6:00 PM
- Demo video: Max 90 seconds
- Free tier services only
- No hardcoded secrets in repo

Success Criteria:
- Deploys successfully to GitHub Pages
- Covers all 13 weeks of course content
- RAG chatbot answers accurately (20 test queries)
- Selected text feature functional
- Runs error-free on fresh Ubuntu 22.04
```

## Response snapshot

The project constitution has been updated to version 1.0.0. This is the initial creation of the constitution. No files require manual follow-up.

I suggest the following commit message: `docs: amend constitution to v1.0.0 (initial creation)`

## Outcome

- ✅ Impact: The project constitution was successfully created and updated, and dependent templates were aligned.
- 🧪 Tests: No new tests were run for this documentation update.
- 📁 Files: The constitution file and three template files were modified.
- 🔁 Next prompts: The user can now proceed with further project development based on the defined constitution.
- 🧠 Reflection: The process of updating the constitution and propagating changes to templates was smooth.

## Evaluation notes (flywheel)

- Failure modes observed: {{FAILURE_MODES}}
- Graders run and results (PASS/FAIL): {{GRADER_RESULTS}}
- Prompt variant (if applicable): {{PROMPT_VARIANT_ID}}
- Next experiment (smallest change to try): {{NEXT_EXPERIMENT}}
