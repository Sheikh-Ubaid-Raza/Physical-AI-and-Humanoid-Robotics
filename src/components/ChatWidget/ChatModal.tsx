import React, { useState, useRef, KeyboardEvent } from 'react';
import { MessageBubble } from './MessageBubble';
import { ChatAPI } from './services/api';

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

interface ChatModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSendMessage: (message: string) => void;
  messages: Message[];
  isLoading: boolean;
  onClear: () => void;
  messagesEndRef: React.RefObject<HTMLDivElement>;
}

export const ChatModal: React.FC<ChatModalProps> = ({
  isOpen,
  onClose,
  onSendMessage,
  messages,
  isLoading,
  onClear,
  messagesEndRef
}) => {
  const [inputValue, setInputValue] = useState('');
  const inputRef = useRef<HTMLInputElement>(null);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (inputValue.trim() && !isLoading) {
      onSendMessage(inputValue);
      setInputValue('');
    }
  };

  const handleKeyDown = (e: KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      if (inputValue.trim() && !isLoading) {
        handleSubmit(e as any); // Type assertion since React.FormEvent and KeyboardEvent are compatible here
      }
    }
  };

  if (!isOpen) return null;

  return (
    <div className="chat-modal-overlay">
      <div className="chat-modal">
        <div className="chat-header">
          <h3>AI Study Assistant</h3>
          <div className="chat-actions">
            <button onClick={onClear} className="clear-btn" title="Clear conversation">
              🗑️
            </button>
            <button onClick={onClose} className="close-btn" title="Close chat">
              ✕
            </button>
          </div>
        </div>

        <div className="chat-messages">
          {messages.length === 0 ? (
            <div className="welcome-message">
              <p>Hello! I'm your AI study assistant for Physical AI & Humanoid Robotics.</p>
              <p>Ask me anything about the textbook content, and I'll provide answers based on the course material.</p>
            </div>
          ) : (
            messages.map((message) => (
              <MessageBubble
                key={message.id}
                role={message.role}
                content={message.content}
                timestamp={message.timestamp}
                sources={message.sources}
              />
            ))
          )}
          {isLoading && (
            <MessageBubble
              role="assistant"
              content=""
              timestamp={new Date()}
              isTyping={true}
            />
          )}
          <div ref={messagesEndRef} />
        </div>

        <form className="chat-input-form" onSubmit={handleSubmit}>
          <input
            ref={inputRef}
            type="text"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyDown={(e) => handleKeyDown(e as any)}
            placeholder="Ask a question about the textbook..."
            disabled={isLoading}
            className="chat-input"
          />
          <button
            type="submit"
            disabled={!inputValue.trim() || isLoading}
            className="send-button"
          >
            {isLoading ? 'Sending...' : '➤'}
          </button>
        </form>
      </div>
    </div>
  );
};