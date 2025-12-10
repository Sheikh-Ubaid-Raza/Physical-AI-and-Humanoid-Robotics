# Feature Specification: RAG Chatbot Advanced Features

**Feature Branch**: `001-rag-chatbot-advanced`
**Created**: 2025-12-10
**Status**: Draft
**Input**: User description: "# RAG Chatbot for Physical AI & Humanoid Robotics Textbook - Part 2 (Advanced Features & Bonus Points)

Target audience: Students learning Physical AI through interactive, personalized Docusaurus textbook
Focus: Advanced features including selected text queries, user authentication, content personalization, and Urdu translation
Success criteria:
- User can highlight any text in the book → right-click/button → ask AI about selection
- User can sign up/sign in with fastapi native-auth, providing software/hardware background
- Logged-in users can personalize chapter content based on their background (beginner/intermediate/advanced)
- Logged-in users can translate chapter content to Urdu with one button click
- All Part 1 features remain functional and enhanced
- Earn up to 200 bonus points (50 per feature category)

Constraints:
- Must build on top of Part 1 implementation (no breaking changes)
- Free tier services: Existing Gemini/Qdrant/Neon quotas
- Tech stack: Same as Part 1 + python-jose + passlib for auth
- Maintain <5s response time for all features
- Mobile-responsive UI for all new features

## CRITICAL: NO OPENAI API USAGE

**I do not have an OpenAI subscription.**

→ Remove all OpenAI API usage (no gpt-4o, no gpt-4, no OPENAI_API_KEY).

→ Use Google Gemini (free tier) for ALL LLM + embeddings.

→ You may use the OpenAI Agents SDK only as an agent framework, configured to run entirely on Gemini models.

### Gemini Requirements

- **LLM:** gemini-2.5-flash (or gemini-2.0-flash-exp if needed for better performance)
- **Embeddings:** embedding-001 (768 dimensions)
- All API calls must use GEMINI_API_KEY
- No OpenAI imports for API/LLM/embeddings—Gemini only

### OpenAI Agents SDK Requirements

- Use latest OpenAI Agents SDK
- Use SDK primitives: Agent, Runner, Sessions, Guardrails, Tools, Handoffs through **Context7** MCP Server
- Configure all models inside SDK to call Gemini, not OpenAI
- Build a RAG-Agent with:
  - Qdrant retrieval tool
  - Gemini embeddings
  - Sessions for chat history
  - Guardrails for relevance

### Project Target

A fully working RAG chatbot inside my Docusaurus textbook, answering questions only from my book.

### Success Criteria (Gemini-Specific)

- Floating chat widget (bottom-right)
- Accurate answers with citations
- Highlight text → "Ask about this" → autofill + prioritized context
- Chat history persists across pages (localStorage + session_id)
- Pre-ingest all /docs/*.md markdown
- Backend deployable on Railway/Render (free)
- Entirely free-tier stack (Gemini + Qdrant + Neon)

## Core Features to Build

### 1. Selected Text Feature (Required - Requirement #2)

- Frontend: Text selection detection on all book pages
- UI: Floating tooltip or context menu when text highlighted
- Backend: POST /api/v1/chat/selection endpoint
- RAG enhancement: Use selected text as context + query
- Response: Explain highlighted concept + link to related chapters
- Example: User highlights "NVIDIA Jetson Orin Nano" → AI explains specs, use cases, related modules

### 2. FastAPI Native Authentication (Bonus +50 points - Requirement #5)

- Backend:
  - POST /api/v1/auth/signup (with background questionnaire)
  - POST /api/v1/auth/signin
  - POST /api/v1/auth/signout
  - GET /api/v1/auth/me (get current user)
  - Use python-jose for JWT tokens
  - Use passlib with bcrypt for password hashing
- Frontend:
  - Signup form with fields: name, email, password, software_background (beginner/intermediate/advanced), hardware_background (none/basic/advanced)
  - Signin form
  - User profile dropdown in navbar
  - Store JWT in localStorage/sessionStorage
  - Include JWT in Authorization header for protected requests
- Database:
  - Neon: users(id, email, name, password_hash, software_background, hardware_background, created_at)
- JWT tokens with 24h expiry
- Protected routes: Personalization and translation require authentication
- Input validation: Email format, password strength (min 8 chars)

### 3. Content Personalization (Bonus +50 points - Requirement #6)

- Frontend: "Personalize Content" button at start of each chapter (only for logged-in users)
- Backend: POST /api/v1/personalize
- Logic:
  - Fetch user's software_background and hardware_background
  - Use Gemini (gemini-2.5-flash) to adjust chapter content:
    - Beginner: More explanations, simpler examples, step-by-step guides
    - Intermediate: Balanced theory and practice
    - Advanced: Skip basics, focus on optimization and advanced concepts
- UI: Replace chapter content dynamically with personalized version
- Cache personalized content per user to avoid regeneration
- Reset button to return to original content

### 4. Urdu Translation (Bonus +50 points - Requirement #7)

- Frontend: "Translate to Urdu" button at start of each chapter (only for logged-in users)
- Backend: POST /api/v1/translate
- Logic:
  - Use Gemini (gemini-2.5-flash) to translate entire chapter from English to Urdu
  - Maintain markdown formatting, code blocks untranslated
  - Preserve technical terms (ROS 2, NVIDIA Isaac, Gazebo) in English
- UI: Replace chapter content with Urdu translation
- Toggle button to switch back to English
- Cache translations per chapter to avoid re-translation costs

### 5. Claude Code Subagents & Skills (Bonus +50 points - Requirement #4)

- Document creation and usage of reusable Claude Code Subagents during development
- Create at least 3 subagents:
  - Backend API Subagent: Handles FastAPI endpoint creation
  - RAG Pipeline Subagent: Manages embeddings and vector operations
  - Frontend Integration Subagent: Handles React component integration
- Create at least 2 Agent Skills:
  - Database Schema Generator: Creates Neon tables
  - Authentication Flow Generator: Implements fastapi native-auth patterns
- Include README documenting how subagents/skills were used
- Show time saved and productivity improvements

## Not Building in Part 2

- Mobile app version
- Advanced analytics dashboard
- Multi-user collaboration features
- Real-time notifications
- Payment/subscription system
- Additional language translations beyond Urdu
- Voice input/output for chatbot
- Offline mode

## Technical Requirements

### Backend

- New endpoints for auth, personalization, translation
- JWT authentication using python-jose (HS256 algorithm)
- Password hashing using passlib with bcrypt
- User background stored in Neon
- Gemini API calls for personalization and translation (gemini-2.5-flash)
- Auth middleware to protect routes
- Dependencies: python-jose[cryptography], passlib[bcrypt], python-multipart, google-generativeai
- **NO OpenAI API imports** - use Gemini for all LLM operations
- Gemini embeddings (embedding-001) for vector operations
- Qdrant collection must use 768 dimensions (Gemini embedding size)

### Frontend

- Text selection detection (window.getSelection())
- Auth forms: Signup and Signin components
- JWT storage in localStorage (with expiry check)
- Protected routes and conditional rendering
- Authorization header: "Bearer {jwt_token}"
- Button components for personalize/translate at chapter starts
- User profile UI in navbar
- Floating chat widget (bottom-right corner)
- Modal interface for chat
- Citations display
- Text-selection → question feature with context prioritization

### Database Schemas

- Neon:
  - users(id, email, name, password_hash, software_background, hardware_background, created_at)
  - personalized_content(id, user_id, chapter_id, content, created_at)
  - translations(id, chapter_id, language, content, created_at)
  - conversations(id, session_id, created_at)
  - messages(id, conversation_id, role, content, sources, timestamp)

### Environment Variables

```
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

### Testing

- Auth flow tests (signup, signin, protected routes, JWT validation)
- Selected text feature tests
- Personalization logic tests
- Translation API tests
- Password hashing and verification tests
- Gemini API integration tests
- RAG pipeline tests with Gemini embeddings
- Target: ~70% backend coverage

## Tech Stack (Free Tier Only)

- **Agent Framework:** OpenAI Agents SDK (configured for Gemini)
- **LLM/Embeddings:** Gemini (gemini-2.5-flash + embedding-001)
- **Vector DB:** Qdrant Cloud free tier (1GB)
- **Database:** Neon Serverless Postgres (3GB free)
- **Backend:** FastAPI
- **Frontend:** Docusaurus + React widget
- **Auth:** FastAPI native auth (python-jose + passlib)
- **Deployment:** Railway/Render (free tier)

## Deliverables

1. Selected text feature: User can highlight text → ask AI → receive contextual explanation
2. FastAPI native authentication (JWT) with signup/signin and background questionnaire
3. Personalization buttons on all chapters (logged-in users only)
4. Urdu translation buttons on all chapters (logged-in users only)
5. Documentation of Claude Code Subagents/Skills usage
6. Updated README with Part 2 setup instructions (including auth setup)
7. Updated demo video (<90 seconds) showing all new features
8. Test suite covering new endpoints and features (auth, personalization, translation)
9. Gemini-integrated RAG system (no OpenAI API code)
10. OpenAI Agents SDK agent configured for Gemini
11. Complete ingestion script (chunk → embed with Gemini → Qdrant)
12. FastAPI backend with /chat + /ingest + /selection + session handling
13. React chat widget with:
    - Floating button
    - Modal interface
    - Citations display
    - Text-selection → question feature
14. Railway/Render deployment guide
15. Basic pytest smoke tests (~70% coverage)

## Success Validation

- Selected text: User highlights "ROS 2 nodes" → AI explains nodes concept + links to Week 3-5
- Auth: User signs up → provides background → logs in → JWT stored → profile shows in navbar
- Protected routes: Accessing /api/v1/personalize without JWT returns 401 Unauthorized
- Personalization: Beginner user clicks "Personalize" → sees simplified explanations (Gemini-generated)
- Translation: User clicks "Translate to Urdu" → chapter appears in Urdu, code blocks in English (Gemini-translated)
- Logout: User logs out → JWT cleared → protected features disabled
- Token expiry: After 24h, user must re-login (JWT expired)
- Subagents: README shows 3+ subagents used, documents time saved
- All features work on mobile devices
- No breaking changes to Part 1 functionality
- Chat widget works with Gemini embeddings (768-dim vectors in Qdrant)
- Accurate answers with citations from book content only
- Chat history persists across page navigation (localStorage + Neon)
- Response time < 5 seconds for all features
- Gemini API calls stay within free tier limits

## Bonus Points Breakdown

- Selected text feature: Required for base 100 points
- FastAPI native authentication: +50 points
- Content personalization: +50 points
- Urdu translation: +50 points
- Claude Code Subagents/Skills documentation: +50 points

**Total possible:** 100 (base) + 200 (bonus) = 300 points (capped at 250 per rubric)

## Important Notes

- **NO OpenAI API usage anywhere** - all LLM/embedding operations use Gemini
- OpenAI Agents SDK is used only as a framework, not for API calls
- Qdrant collection must be 768 dimensions (Gemini embedding-001 size, NOT 1536)
- All existing Part 1 code must be refactored to use Gemini instead of OpenAI
- Ensure Gemini API free tier limits are respected (rate limiting may be needed)
- Test thoroughly with Gemini models before deployment"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Selected Text Feature (Priority: P1)

As a student reading the Physical AI textbook, I want to highlight any text in the book and ask the AI about it so that I can get immediate contextual explanations and related content.

**Why this priority**: This is the core required feature that enables interactive learning and is essential for the base 100 points.

**Independent Test**: Can be fully tested by highlighting text and clicking the "Ask about this" button, which should trigger the AI to explain the highlighted concept with citations to related chapters.

**Acceptance Scenarios**:

1. **Given** I am reading a chapter in the textbook, **When** I highlight text and click the context menu button, **Then** the chat widget should appear with the highlighted text pre-filled as a question.

2. **Given** I have highlighted text in the textbook, **When** I click the floating tooltip, **Then** the AI should provide an explanation of the highlighted concept with links to related chapters.

---
### User Story 2 - User Authentication (Priority: P2)

As a student, I want to sign up and sign in to the system with my software/hardware background so that I can access personalized content and translations.

**Why this priority**: Authentication is required to access the personalization and translation features (bonus points), and it's a prerequisite for other advanced features.

**Independent Test**: Can be fully tested by signing up with email, password, and background information, then signing in to access the system with a JWT token.

**Acceptance Scenarios**:

1. **Given** I am a new user, **When** I fill out the signup form with valid information, **Then** I should be able to create an account and receive a JWT token.

2. **Given** I have an account, **When** I sign in with correct credentials, **Then** I should be authenticated and able to access protected features.

---
### User Story 3 - Content Personalization (Priority: P3)

As a logged-in student with a specific background level, I want to personalize chapter content based on my software/hardware experience so that the material is adjusted to my knowledge level.

**Why this priority**: This is a bonus feature that enhances the learning experience but depends on authentication being implemented first.

**Independent Test**: Can be fully tested by logging in, clicking the "Personalize Content" button, and seeing the chapter content adjusted based on my background level.

**Acceptance Scenarios**:

1. **Given** I am a logged-in user with beginner background, **When** I click the personalization button, **Then** the chapter content should be simplified with more explanations and step-by-step guides.

---
### User Story 4 - Urdu Translation (Priority: P4)

As a logged-in student who prefers Urdu, I want to translate chapter content to Urdu with one button click so that I can understand the material in my preferred language.

**Why this priority**: This is a bonus feature that enhances accessibility but depends on authentication being implemented first.

**Independent Test**: Can be fully tested by logging in, clicking the "Translate to Urdu" button, and seeing the chapter content translated while preserving technical terms in English.

**Acceptance Scenarios**:

1. **Given** I am a logged-in user, **When** I click the Urdu translation button, **Then** the chapter should appear in Urdu with code blocks and technical terms preserved in English.

---
### User Story 5 - Claude Code Subagents & Skills Documentation (Priority: P5)

As a developer, I want to document the Claude Code Subagents and Skills used during development so that the development process and productivity improvements are captured.

**Why this priority**: This is a documentation requirement for bonus points but doesn't directly impact user functionality.

**Independent Test**: Can be fully tested by reviewing the documentation that shows at least 3 subagents and 2 skills used during development.

**Acceptance Scenarios**:

1. **Given** the development process is complete, **When** I review the documentation, **Then** I should see evidence of at least 3 subagents and 2 skills that were used.

---
### Edge Cases

- What happens when a user tries to access protected features without authentication?
- How does the system handle expired JWT tokens?
- What happens when the Gemini API is unavailable or rate-limited?
- How does the system handle invalid text selections or empty selections?
- What happens when a user tries to translate content that is already in Urdu?
- How does the system handle very long chapter content for personalization or translation?

## Requirements *(mandatory)*

### Functional Requirements
*(All functional requirements MUST align with the Architecture, Security, and Code Quality standards defined in the project constitution.)*

- **FR-001**: System MUST allow users to highlight text in the textbook and trigger an AI explanation of the selected content
- **FR-002**: System MUST provide a signup form that collects name, email, password, software background, and hardware background
- **FR-003**: System MUST authenticate users via JWT tokens with 24-hour expiration
- **FR-004**: System MUST hash passwords using bcrypt for security
- **FR-005**: System MUST provide a personalization endpoint that adjusts chapter content based on user background
- **FR-006**: System MUST provide a translation endpoint that converts chapter content to Urdu while preserving technical terms
- **FR-007**: System MUST store user information securely in the Neon database
- **FR-008**: System MUST maintain response times under 5 seconds for all features
- **FR-009**: System MUST use Gemini API for all LLM and embedding operations (no OpenAI)
- **FR-010**: System MUST preserve markdown formatting during translation and personalization
- **FR-011**: System MUST cache personalized content and translations to avoid regeneration costs
- **FR-012**: System MUST validate email format and password strength (minimum 8 characters)
- **FR-013**: System MUST support mobile-responsive UI for all new features
- **FR-014**: System MUST maintain all existing Part 1 functionality without breaking changes
- **FR-015**: System MUST use 768-dimensional embeddings from Gemini for vector operations

### Key Entities *(include if feature involves data)*

- **User**: Represents a student with email, name, hashed password, software background (beginner/intermediate/advanced), hardware background (none/basic/advanced), and creation timestamp
- **PersonalizedContent**: Represents customized chapter content generated for a specific user based on their background, linked to both user and chapter
- **Translation**: Represents translated chapter content in a specific language, linked to the original chapter
- **JWT Token**: Represents an authenticated user session with 24-hour expiration

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can highlight any text in the textbook and receive an AI explanation within 5 seconds
- **SC-002**: Users can complete account signup with background information in under 2 minutes
- **SC-003**: Authenticated users can personalize chapter content based on their background with a single button click
- **SC-004**: Authenticated users can translate chapter content to Urdu with preserved formatting in under 10 seconds
- **SC-005**: All features maintain response times under 5 seconds even with Gemini API usage
- **SC-006**: The system works without any OpenAI API usage, using only Gemini for LLM operations
- **SC-007**: All existing Part 1 functionality continues to work without breaking changes
- **SC-008**: Mobile users can access all new features with responsive UI elements
- **SC-009**: JWT tokens expire after 24 hours requiring re-authentication
- **SC-010**: The system stays within free tier limits for all services (Gemini, Qdrant, Neon)
