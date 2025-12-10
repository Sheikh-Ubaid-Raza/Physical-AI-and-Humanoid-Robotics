---
id: 2
title: "Implement ChatWidget on Landing Page"
stage: "general"
date: "2025-12-10"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "001-rag-chatbot"
user: "user"
command: "implement rag chatbot chatwidget component in main landing page"
labels: ["chatwidget", "ui", "frontend", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "src/theme/Root.tsx"
  - "src/components/ChatWidget/ChatWidget.tsx"
  - "src/components/ChatWidget/ChatModal.tsx"
  - "src/components/ChatWidget/MessageBubble.tsx"
  - "src/components/ChatWidget/ChatWidget.module.css"
  - "src/components/ChatWidget/services/api.ts"
  - ".env"
tests: []
---

# Implement ChatWidget on Landing Page

## User Prompt
```
i was created the rag chabot chatwidget component, implement it in main landing page in the bottom right corner
```

## Response
Successfully implemented the RAG chatbot ChatWidget component on the main landing page in the bottom right corner. The implementation included:

1. **Created global Root component**:
   - Created `src/theme/Root.tsx` to wrap the entire application
   - This ensures the ChatWidget is available on all pages, including the landing page

2. **Copied ChatWidget components to main src directory**:
   - Copied ChatWidget, ChatModal, MessageBubble components from `docs/src` to `src/components`
   - Copied CSS styles and API service files
   - Ensured all import paths are correct for the new location

3. **Updated API configuration**:
   - Fixed backend API URL to use port 8001 instead of 8000 (to match running backend)
   - Updated the ChatWidget to use the ChatAPI service instead of direct fetch calls
   - Added REACT_APP_API_BASE_URL to .env file for proper configuration

4. **Ensured proper positioning**:
   - The ChatWidget uses CSS positioning to appear in the bottom right corner (fixed position with bottom: 30px, right: 30px)
   - Responsive design ensures it works on different screen sizes

## Outcome
The ChatWidget is now available on all pages of the Docusaurus site, including the main landing page, positioned in the bottom right corner as requested. It connects to the backend API running on localhost:8001 and maintains conversation history across sessions.