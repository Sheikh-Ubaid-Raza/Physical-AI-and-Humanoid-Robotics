# Implementation Plan: [FEATURE]

**Branch**: `[###-feature-name]` | **Date**: [DATE] | **Spec**: [link]
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

[Extract from feature spec: primary requirement + technical approach from research]

## Technical Context

<!--
  ACTION REQUIRED: Replace the content in this section with the technical details
  for the project. The structure here is presented in advisory capacity to guide
  the iteration process.
-->

**Language/Version**: [e.g., Python 3.11, Swift 5.9, Rust 1.75 or NEEDS CLARIFICATION]  
**Primary Dependencies**: [e.g., FastAPI, UIKit, LLVM or NEEDS CLARIFICATION]  
**Storage**: [if applicable, e.g., PostgreSQL, CoreData, files or N/A]  
**Testing**: [e.g., pytest, XCTest, cargo test or NEEDS CLARIFICATION]  
**Target Platform**: [e.g., Linux server, iOS 15+, WASM or NEEDS CLARIFICATION]
**Project Type**: [single/web/mobile - determines source structure]  
**Performance Goals**: [domain-specific, e.g., 1000 req/s, 10k lines/sec, 60 fps or NEEDS CLARIFICATION]  
**Constraints**: [domain-specific, e.g., <200ms p95, <100MB memory, offline-capable or NEEDS CLARIFICATION]  
**Scale/Scope**: [domain-specific, e.g., 10k users, 1M LOC, 50 screens or NEEDS CLARIFICATION]

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
- **RAG Chatbot**: Embeddings MUST use OpenAI text-embedding-3-small. Chunk size: 500-1000 tokens with 100-token overlap. Top-5 retrieval, confidence threshold: 0.6. Support for selected text queries and inclusion of chapter references in responses are REQUIRED.

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)
<!--
  ACTION REQUIRED: Replace the placeholder tree below with the concrete layout
  for this feature. Delete unused options and expand the chosen structure with
  real paths (e.g., apps/admin, packages/something). The delivered plan must
  not include Option labels.
-->

```text
# [REMOVE IF UNUSED] Option 1: Single project (DEFAULT)
src/
├── models/
├── services/
├── cli/
└── lib/

tests/
├── contract/
├── integration/
└── unit/

# [REMOVE IF UNUSED] Option 2: Web application (when "frontend" + "backend" detected)
backend/
├── src/
│   ├── models/
│   ├── services/
│   └── api/
└── tests/

frontend/
├── src/
│   ├── components/
│   ├── pages/
│   └── services/
└── tests/

# [REMOVE IF UNUSED] Option 3: Mobile + API (when "iOS/Android" detected)
api/
└── [same as backend above]

ios/ or android/
└── [platform-specific structure: feature modules, UI flows, platform tests]
```

**Structure Decision**: [Document the selected structure and reference the real
directories captured above]

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
