# Physical AI & Humanoid Robotics Textbook - Frontend

This directory contains the Docusaurus-based frontend for the Physical AI & Humanoid Robotics textbook website. The site includes an integrated RAG chatbot widget that allows students to ask questions about the course content.

## Project Structure

```
docs/
├── src/
│   ├── components/
│   │   └── ChatWidget/          # Chatbot widget components
│   │       ├── ChatWidget.tsx   # Main chat widget component
│   │       ├── ChatModal.tsx    # Modal version of the chat interface
│   │       ├── MessageBubble.tsx # Individual message display component
│   │       ├── ChatWidget.module.css # Styling for the chat components
│   │       └── services/        # Chat-related services and utilities
│   ├── pages/                   # Custom pages
│   └── theme/                   # Custom theme components
├── docs/                        # Course content (week-01 to week-13)
├── tutorial-basics/            # Docusaurus tutorial basics
├── tutorial-extras/            # Docusaurus tutorial extras
├── hardware-requirements.md    # Hardware requirements documentation
├── intro.md                    # Introduction page
└── package.json               # Frontend dependencies and scripts
```

## Chat Widget Usage

The chat widget is integrated throughout the textbook pages and provides the following functionality:

### Features
- **Context-aware responses**: The chatbot understands the current page content and provides relevant answers
- **Citation support**: Responses include citations to specific chapters, modules, and weeks
- **Session persistence**: Conversations are maintained across page navigation
- **Responsive design**: Works on desktop and mobile devices

### How to Use the Chat Widget
1. Navigate to any chapter page in the textbook
2. Look for the floating chat icon in the bottom-right corner of the page
3. Click the icon to open the chat widget
4. Type your question about the textbook content
5. The chatbot will respond with information from the textbook and provide citations

### Widget Integration
The chat widget is automatically included on all textbook pages via the Docusaurus theme. It's implemented using:
- React TypeScript components
- WebSocket connections to the backend API
- Local session management using browser storage

### Customization
To customize the chat widget behavior:
1. Modify the components in `src/components/ChatWidget/`
2. Update the API endpoint configuration in the service files
3. Adjust the styling in `ChatWidget.module.css`

### API Integration
The chat widget communicates with the backend API at:
- Chat endpoint: `/api/v1/chat`
- Requires session management and proper CORS configuration
- Handles authentication and rate limiting

## Local Development

To run the frontend locally:

```bash
# Install dependencies
npm install

# Start development server
npm start
```

The site will be available at `http://localhost:3000`.

## Build

To build the static site:

```bash
npm run build
```

## Deployment

The frontend is automatically deployed to GitHub Pages when changes are pushed to the main branch via GitHub Actions.

## Available Scripts

- `npm start` - Start development server
- `npm run build` - Build static site
- `npm run serve` - Serve built site locally
- `npm run deploy` - Deploy to GitHub Pages
- `npm run clear` - Clear build cache
- `npm test` - Run frontend tests
- `npm run typecheck` - Check TypeScript types

## Environment Variables

The frontend doesn't require specific environment variables, but it expects the backend API to be available at the configured endpoint.

## Troubleshooting

### Chat Widget Not Loading
- Verify the backend API is running and accessible
- Check browser console for CORS or network errors
- Ensure the API endpoint is correctly configured

### Session Issues
- Clear browser cache and cookies if experiencing session problems
- Check that the session management service is working correctly

### Styling Issues
- Verify CSS modules are properly loaded
- Check for conflicting styles from Docusaurus theme