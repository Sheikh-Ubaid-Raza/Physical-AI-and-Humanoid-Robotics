# RAG Chatbot for Physical AI & Humanoid Robotics Textbook

## Feature Overview

### Description
A RAG (Retrieval Augmented Generation) chatbot that allows students to ask questions about the Physical AI & Humanoid Robotics curriculum and receive accurate answers based on the textbook content. The chatbot will be embedded as a widget in the Docusaurus frontend and will retrieve relevant information from a Qdrant vector database.

### Target Audience
Students learning Physical AI through the interactive Docusaurus textbook who want to ask questions and get immediate answers from the course content.

### Business Value
- Improves student learning experience by providing instant answers to questions
- Reduces instructor workload by handling common questions automatically
- Increases engagement with the curriculum through interactive learning
- Provides 24/7 support for students across different time zones

## User Scenarios & Testing

### Primary User Scenarios

1. **Student asks a question about course content**
   - Student opens the textbook website
   - Student sees the chat widget in the bottom-right corner
   - Student types a question about Physical AI concepts (e.g., "What is ROS 2?")
   - Chatbot processes the query and returns an accurate answer citing specific chapters
   - Student can continue the conversation with follow-up questions

2. **Student continues a conversation with follow-up questions**
   - Student has already asked a question and received a response
   - Student asks a follow-up question that references the previous context
   - Chatbot maintains conversation context and provides relevant responses
   - Conversation history is preserved during the same session

3. **Student navigates between pages while maintaining chat history**
   - Student starts a conversation on Week 1 content
   - Student navigates to Week 5 content
   - Chat history remains available in the same session
   - Student can continue the conversation with context from previous pages

### Edge Cases
- Student asks off-topic questions (should be redirected to course content)
- Student asks questions about content not covered in the textbook
- Network connectivity issues during chat interaction
- Multiple tabs open with the same chat session

## Functional Requirements

### FR-1: Chat Interface
- The system shall provide a chat widget that appears on all pages of the textbook
- The chat widget shall be positioned in the bottom-right corner of the screen
- The chat interface shall display messages in a clear, readable format
- The system shall support text input with a submit button and Enter key
- The system shall display typing indicators when the chatbot is processing

### FR-2: Question Processing
- The system shall accept text-based questions from users
- The system shall process questions using a RAG pipeline that searches course content
- The system shall return answers that are directly based on the textbook content
- The system shall include citations to specific chapters, weeks, or modules in responses
- The system shall handle follow-up questions with proper context retention

### FR-3: Content Retrieval
- The system shall maintain a vector database of all textbook content
- The system shall retrieve the top 5 most relevant content chunks for each query
- The system shall use embeddings to match user queries with textbook content
- The system shall return results within 5 seconds of query submission
- The system shall handle content chunks of 500-1000 tokens with 100-token overlap

### FR-4: Session Management
- The system shall generate a unique session ID for each user visit
- The system shall maintain chat history within the same session
- The system shall preserve conversation context across page navigation
- The system shall not persist chat history between different browser sessions
- The system shall handle session timeouts gracefully

### FR-5: Content Guardrails
- The system shall restrict responses to content from the Physical AI textbook only
- The system shall politely decline to answer off-topic questions
- The system shall provide helpful redirects when questions are outside scope
- The system shall maintain academic integrity by citing sources
- The system shall not generate hallucinated information

### FR-6: Backend API
- The system shall provide a health check endpoint accessible via GET /api/v1/health
- The system shall provide a chat endpoint accessible via POST /api/v1/chat
- The system shall provide an ingestion endpoint accessible via POST /api/v1/ingest
- The system shall handle API requests with proper error responses
- The system shall implement proper CORS configuration for GitHub Pages

## Non-Functional Requirements

### Performance
- The system shall respond to queries within 5 seconds
- The system shall handle concurrent users without degradation
- The system shall maintain consistent response times regardless of content size

### Security
- The system shall validate all user inputs to prevent injection attacks
- The system shall implement proper authentication for admin endpoints
- The system shall protect against excessive API usage with rate limiting
- The system shall encrypt sensitive data in transit

### Reliability
- The system shall maintain 99% uptime during educational hours
- The system shall handle backend failures gracefully with user-friendly messages
- The system shall implement proper error logging for debugging
- The system shall have a recovery time of less than 1 hour for failures

### Scalability
- The system shall support up to 100 concurrent users
- The system shall handle increasing content volume without performance degradation
- The system shall scale horizontally with additional resources

## Key Entities

### Conversation
- session_id: Unique identifier for the user session
- created_at: Timestamp of when the conversation started
- messages: Collection of messages in the conversation

### Message
- conversation_id: Reference to the parent conversation
- role: Role of the message sender (user or assistant)
- content: Text content of the message
- sources: List of content sources used to generate the response
- timestamp: When the message was created

### Content Chunk
- module: Which module/chapter the content belongs to
- week: Which week of the curriculum
- chapter_title: Title of the specific chapter
- content: The actual text content
- url: URL where the content can be found in the textbook
- embedding: Vector representation of the content

## Success Criteria

### Quantitative Measures
- 95% of questions receive relevant answers from textbook content
- 100% of questions about textbook content receive accurate answers
- Response time under 5 seconds for 95% of queries
- 99% uptime during educational hours
- 20 test queries all return accurate answers from book content

### Qualitative Measures
- Students find the chatbot helpful for understanding course material
- Students report improved learning experience with chatbot assistance
- Students can ask follow-up questions and maintain context
- Chatbot responses include proper citations to textbook content
- Chatbot refuses to answer off-topic questions appropriately

### Business Outcomes
- Increased student engagement with the textbook content
- Reduced time spent by instructors answering common questions
- Improved student satisfaction with the learning platform
- Successful deployment on GitHub Pages frontend and Railway backend

## Assumptions

- Students have basic familiarity with chat interfaces
- The textbook content is comprehensive enough to answer most questions
- Students will use the chatbot primarily during study sessions
- Network connectivity is generally stable for users
- The vector database will be populated with all textbook content before launch
- Free tier services (OpenAI, Qdrant Cloud, Neon Postgres, Railway) will be sufficient for initial deployment

## Dependencies

- OpenAI Agents SDK for language model processing
- Qdrant vector database for content retrieval
- Neon Postgres for chat history storage
- Docusaurus frontend framework
- FastAPI backend framework
- GitHub Pages for frontend hosting
- Railway for backend deployment

## Constraints

- Must use free tier services only (OpenAI, Qdrant Cloud 1GB, Neon Postgres, Railway $5 credit)
- Backend must use Python 3.10+ with FastAPI
- Frontend must integrate with existing Docusaurus v3.x structure
- Responses must be limited to textbook content only (no external knowledge)
- System must deploy to GitHub Pages (frontend) and Railway (backend)
- Embeddings must use 500-1000 token chunks with 100-token overlap