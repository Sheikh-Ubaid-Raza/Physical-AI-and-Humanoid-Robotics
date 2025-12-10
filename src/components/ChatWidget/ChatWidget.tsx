import React, { useState, useEffect, useRef } from 'react';
import { ChatModal } from './ChatModal';
import { ChatAPIInstance } from './api';
import './ChatWidget.module.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: Array<{
    module: string;
    week: string;
    chapter_title: string;
    url: string;
  }>;
}

export const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [sessionID, setSessionID] = useState<string>('');
  const [isLoading, setIsLoading] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  // Initialize session ID on component mount with persistent session management
  useEffect(() => {
    // Check if we have a session ID in localStorage
    const storedSessionID = localStorage.getItem('chatbot_session_id');
    if (storedSessionID) {
      setSessionID(storedSessionID);
    } else {
      // Create a new session ID
      const newSessionID = generateSessionID();
      setSessionID(newSessionID);
      localStorage.setItem('chatbot_session_id', newSessionID);
    }

    // Load conversation history if it exists
    loadConversationHistory(storedSessionID || '');

    // Set up session synchronization across pages/tabs
    syncSessionAcrossPages();
  }, []);

  // Implement persistent session management using localStorage or cookies
  const getSessionId = (): string => {
    let sessionId = localStorage.getItem('chatbot_session_id');
    if (!sessionId) {
      sessionId = generateSessionID();
      localStorage.setItem('chatbot_session_id', sessionId);
    }
    return sessionId;
  };

  const setSessionId = (id: string) => {
    localStorage.setItem('chatbot_session_id', id);
    setSessionID(id);
  };

  // Handle session restoration on page load
  const restoreSessionOnPageLoad = () => {
    const savedSessionId = getSessionId();
    if (savedSessionId && savedSessionId !== sessionID) {
      setSessionId(savedSessionId);
      loadConversationHistory(savedSessionId);
    }
  };

  // Create session synchronization between pages
  const syncSessionAcrossPages = () => {
    // Listen for storage events to sync session changes across tabs/windows
    window.addEventListener('storage', (event) => {
      if (event.key === 'chatbot_session_id' && event.newValue) {
        // Session ID changed in another tab/window, update this one
        setSessionID(event.newValue);
        loadConversationHistory(event.newValue);
      } else if (event.key?.startsWith('chatbot_conversation_') && event.newValue) {
        // Conversation data changed in another tab/window
        try {
          const data = JSON.parse(event.newValue);
          if (data.sessionID === sessionID) {
            // Load the conversation from the other tab/window
            const conversationMessages = data.messages.map((msg: any) => ({
              ...msg,
              timestamp: new Date(msg.timestamp)
            }));
            setMessages(conversationMessages);
          }
        } catch (error) {
          console.error('Error syncing conversation data:', error);
        }
      }
    });
  };

  // Add session timeout and cleanup functionality
  const setupSessionTimeout = () => {
    // Session timeout in milliseconds (e.g., 24 hours)
    const SESSION_TIMEOUT = 24 * 60 * 60 * 1000; // 24 hours

    // Check for expired sessions periodically
    const interval = setInterval(() => {
      const lastActivityStr = localStorage.getItem('chatbot_last_activity');
      if (lastActivityStr) {
        const lastActivity = new Date(lastActivityStr).getTime();
        const now = new Date().getTime();

        if (now - lastActivity > SESSION_TIMEOUT) {
          // Session has timed out, clear the session data
          localStorage.removeItem('chatbot_session_id');
          localStorage.removeItem(`chatbot_conversation_${sessionID}`);
          localStorage.removeItem('chatbot_last_activity');

          // Create a new session
          const newSessionID = generateSessionID();
          setSessionID(newSessionID);
          localStorage.setItem('chatbot_session_id', newSessionID);
          setMessages([]);
        }
      }
    }, 60 * 1000); // Check every minute

    // Clean up interval on component unmount
    return () => clearInterval(interval);
  };

  // Update last activity timestamp
  const updateLastActivity = () => {
    localStorage.setItem('chatbot_last_activity', new Date().toISOString());
  };

  // Implement frontend session validation and refresh
  const validateAndRefreshSession = () => {
    const sessionId = localStorage.getItem('chatbot_session_id');
    const lastActivityStr = localStorage.getItem('chatbot_last_activity');

    if (!sessionId) {
      // No session exists, create a new one
      const newSessionId = generateSessionID();
      localStorage.setItem('chatbot_session_id', newSessionId);
      setSessionID(newSessionId);
      return newSessionId;
    }

    if (lastActivityStr) {
      const lastActivity = new Date(lastActivityStr).getTime();
      const now = new Date().getTime();
      const SESSION_TIMEOUT = 24 * 60 * 60 * 1000; // 24 hours

      if (now - lastActivity > SESSION_TIMEOUT) {
        // Session has expired, create a new one
        localStorage.removeItem('chatbot_session_id');
        localStorage.removeItem(`chatbot_conversation_${sessionId}`);
        localStorage.removeItem('chatbot_last_activity');

        const newSessionId = generateSessionID();
        localStorage.setItem('chatbot_session_id', newSessionId);
        setSessionID(newSessionId);
        setMessages([]);
        return newSessionId;
      }
    }

    // Session is valid, update activity
    updateLastActivity();
    return sessionId;
  };

  // Initialize session timeout when component mounts
  useEffect(() => {
    setupSessionTimeout();
    updateLastActivity();

    // Add page navigation event handlers for session management
    const handleBeforeUnload = () => {
      // Save current state before page unload
      updateLastActivity();
    };

    const handleVisibilityChange = () => {
      if (!document.hidden) {
        // Tab became visible again, validate session
        validateAndRefreshSession();
      }
    };

    // Add event listeners
    window.addEventListener('beforeunload', handleBeforeUnload);
    document.addEventListener('visibilitychange', handleVisibilityChange);

    // Cleanup event listeners on component unmount
    return () => {
      window.removeEventListener('beforeunload', handleBeforeUnload);
      document.removeEventListener('visibilitychange', handleVisibilityChange);
    };
  }, []);

  // Implement session-based conversation persistence
  const saveConversationToLocalStorage = (messages: Message[]) => {
    if (sessionID) {
      const conversationData = {
        messages,
        timestamp: new Date().toISOString(),
        sessionID
      };
      localStorage.setItem(`chatbot_conversation_${sessionID}`, JSON.stringify(conversationData));
    }
  };

  const loadConversationFromLocalStorage = (): Message[] => {
    if (sessionID) {
      const stored = localStorage.getItem(`chatbot_conversation_${sessionID}`);
      if (stored) {
        try {
          const data = JSON.parse(stored);
          // Return messages with proper Date objects
          return data.messages.map((msg: any) => ({
            ...msg,
            timestamp: new Date(msg.timestamp)
          }));
        } catch (error) {
          console.error('Error parsing stored conversation:', error);
          return [];
        }
      }
    }
    return [];
  };

  // Scroll to bottom of messages when messages change
  useEffect(() => {
    scrollToBottom();
    updateLastActivity(); // Update activity when messages change
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const generateSessionID = (): string => {
    return 'session_' + Date.now().toString(36) + Math.random().toString(36).substr(2, 5);
  };

  const loadConversationHistory = async (sessionId: string) => {
    // First, try to load from localStorage
    const localMessages = loadConversationFromLocalStorage();
    if (localMessages.length > 0) {
      setMessages(localMessages);
      return;
    }

    // In a real implementation, we would fetch conversation history from the backend
    // For now, we'll just initialize with an empty array
    setMessages([]);
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const sendMessage = async (message: string) => {
    if (!message.trim() || isLoading) return;

    // Update last activity timestamp
    updateLastActivity();

    // Add user message to UI immediately
    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: message,
      timestamp: new Date(),
    };

    setMessages(prev => {
      const updated = [...prev, userMessage];
      saveConversationToLocalStorage(updated);
      return updated;
    });
    setIsLoading(true);

    try {
      // Prepare conversation history for context (last 5 messages for context)
      const conversationHistory = messages.slice(-5).map(msg => ({
        role: msg.role,
        content: msg.content
      }));

      // Use the ChatAPI service to call the backend
      const data = await ChatAPIInstance.chat({
        message: message,
        session_id: sessionID,
        conversation_context: conversationHistory,  // Add conversation context to payload
      });

      // Add assistant message to UI
      const assistantMessage: Message = {
        id: data.message_id,
        role: 'assistant',
        content: data.response,
        timestamp: new Date(),
        sources: data.sources,
      };

      setMessages(prev => {
        const updated = [...prev, assistantMessage];
        saveConversationToLocalStorage(updated);
        return updated;
      });
    } catch (error) {
      console.error('Error sending message:', error);

      // Add error message to UI
      const errorMessage: Message = {
        id: Date.now().toString(),
        role: 'assistant',
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date(),
      };

      setMessages(prev => {
        const updated = [...prev, errorMessage];
        saveConversationToLocalStorage(updated);
        return updated;
      });
    } finally {
      setIsLoading(false);
    }
  };

  const clearConversation = () => {
    setMessages([]);
    // In a real implementation, we might want to clear the conversation on the backend too
  };

  return (
    <>
      {/* Floating chat button */}
      {!isOpen && (
        <button
          className="chat-widget-button"
          onClick={toggleChat}
          aria-label="Open chat"
        >
          <div className="chat-icon">💬</div>
        </button>
      )}

      {/* Chat modal */}
      {isOpen && (
        <ChatModal
          isOpen={isOpen}
          onClose={() => setIsOpen(false)}
          onSendMessage={sendMessage}
          messages={messages}
          isLoading={isLoading}
          onClear={clearConversation}
          messagesEndRef={messagesEndRef}
        />
      )}
    </>
  );
};