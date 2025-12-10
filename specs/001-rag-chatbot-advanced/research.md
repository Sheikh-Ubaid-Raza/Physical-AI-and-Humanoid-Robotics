# Research: RAG Chatbot Advanced Features

## Decision: Gemini Model Selection

**Decision**: Use gemini-2.5-flash for all LLM operations
**Rationale**:
- Cost effective: Most economical option in Gemini family
- Performance: Good balance of speed and accuracy for educational content
- Quota management: Efficient usage of free tier limits
- Sufficient for required tasks: Text selection explanations, personalization, and Urdu translation

**Alternatives considered**:
- gemini-2.0-flash-exp: Slightly better performance but potentially higher cost/quota usage
- gemini-pro: More expensive, unnecessary for this use case

## Decision: JWT Storage Strategy

**Decision**: Store JWT in localStorage with proper security measures
**Rationale**:
- Persistence: Tokens survive browser refreshes and tabs
- Accessibility: JavaScript can access tokens for API calls
- Security: Implement proper httpOnly alternative for sensitive operations if needed

**Alternatives considered**:
- sessionStorage: More secure but tokens lost on refresh
- httpOnly cookies: More secure but complex for SPA implementation

## Decision: Qdrant Collection Migration

**Decision**: Recreate Qdrant collection with 768 dimensions (instead of adapting)
**Rationale**:
- Clean migration: Avoid complex adapter patterns
- Performance: Direct compatibility with Gemini embeddings
- Future-proof: Aligns with new architecture
- Simpler implementation: No conversion layer needed

## Decision: Text Selection UI Pattern

**Decision**: Floating tooltip approach for text selection
**Rationale**:
- UX: Non-intrusive, familiar pattern to users
- Mobile compatibility: Can be adapted for touch interfaces
- Visibility: Always accessible when needed
- Clean design: Doesn't clutter the page

**Alternatives considered**:
- Context menu: May conflict with browser context menus
- Dedicated button: Less contextual and discoverable

## Decision: Personalization Caching Strategy

**Decision**: Database caching (Neon) for personalized content
**Rationale**:
- Cost management: Reduce Gemini API calls for repeated personalizations
- Persistence: Caching survives user sessions
- Scalability: Distributed caching potential
- Simplicity: Leverage existing database infrastructure

**Alternatives considered**:
- In-memory caching: Faster but doesn't persist and uses server resources

## Decision: Translation Approach

**Decision**: Full chapter translation with caching
**Rationale**:
- User experience: Complete chapter in target language
- Performance: Cached translations for subsequent access
- API quota: Minimize repeated Gemini calls through caching
- Consistency: Full context preserved in translation

**Alternatives considered**:
- On-demand section translation: More granular but could lead to inconsistent experiences

## Decision: OpenAI Agents SDK Integration with Gemini

**Decision**: Configure OpenAI Agents SDK to work with Gemini via API adapter pattern
**Rationale**:
- Framework reuse: Leverage existing SDK features (Sessions, Guardrails, Tools)
- Consistency: Maintain familiar development patterns
- Compatibility: Work within existing codebase structure
- Flexibility: Can swap models without changing core logic

## Decision: Protected Route Implementation

**Decision**: Use FastAPI dependency injection with JWT validation for protected routes
**Rationale**:
- Clean separation: Security logic separate from business logic
- Reusability: Same dependency can be used across multiple routes
- Maintainability: Centralized authentication logic
- Performance: Fast validation without repeating code

**Alternatives considered**:
- Decorators: More complex to maintain and test

## Decision: Session Management

**Decision**: Client-side session management with server-side validation
**Rationale**:
- Scalability: Reduces server memory requirements
- Simplicity: Leverages JWT stateless nature
- Security: Server validates tokens on protected endpoints
- Performance: Reduces database lookups for session state

**Alternatives considered**:
- Server-side sessions: More secure but requires more server resources and complexity

## Architecture Phases Implementation Strategy

**Phase 1: Infrastructure Setup**
- Update embedding utilities for Gemini compatibility
- Recreate Qdrant collection with 768 dimensions
- Update database schemas for new entities (users, personalized_content, translations)
- Configure environment variables for Gemini API

**Phase 2: Authentication System**
- Implement JWT-based auth with python-jose
- Create password hashing with passlib/bcrypt
- Build auth middleware for protected routes
- Frontend auth forms and JWT storage

**Phase 3: Selected Text Feature**
- Implement text selection detection
- Create floating tooltip UI
- Build selection endpoint with priority context

**Phase 4: Content Personalization**
- Develop personalization service using Gemini
- Create caching mechanism for personalized content

**Phase 5: Urdu Translation**
- Build translation service with Gemini
- Preserve markdown formatting and technical terms

**Phase 6: Integration & Testing**
- Ensure no breaking changes to Part 1
- Test all new features together
- Validate performance requirements

## Risk Mitigation Strategies

- **Gemini API rate limits**: Implement client-side rate limiting and request queuing
- **Qdrant dimension mismatch**: Verify 768-dim vectors before full migration with comprehensive testing
- **JWT security**: Use strong secret keys, implement proper token validation
- **Translation accuracy**: Include human review process for critical content
- **Mobile UX**: Implement responsive design with touch-friendly interfaces
- **Breaking Part 1**: Use feature flags to enable Part 2 features independently