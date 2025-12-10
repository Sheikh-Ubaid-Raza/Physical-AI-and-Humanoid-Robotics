import React from 'react';

interface Message {
  id?: string;
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

interface MessageBubbleProps {
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: Array<{
    module: string;
    week: string;
    chapter_title: string;
    url: string;
  }>;
  isTyping?: boolean;
}

export const MessageBubble: React.FC<MessageBubbleProps> = ({
  role,
  content,
  timestamp,
  sources,
  isTyping = false
}) => {
  const isUser = role === 'user';

  // Format timestamp as HH:MM
  const timeString = timestamp.toLocaleTimeString([], {
    hour: '2-digit',
    minute: '2-digit'
  });

  if (isTyping) {
    return (
      <div className={`message-bubble assistant typing`}>
        <div className="typing-indicator" data-testid="typing-indicator">
          <span></span>
          <span></span>
          <span></span>
        </div>
      </div>
    );
  }

  return (
    <div className={`message-bubble ${role} ${isUser ? 'user' : 'assistant'}`}>
      <div className="message-content">
        {content.split('\n').map((line, i) => (
          <p key={i}>{line}</p>
        ))}
      </div>

      {sources && sources.length > 0 && (
        <div className="message-sources">
          <strong>Sources:</strong>
          <ul>
            {sources.map((source, index) => (
              <li key={index}>
                <a
                  href={source.url}
                  target="_blank"
                  rel="noopener noreferrer"
                  onClick={(e) => e.stopPropagation()}
                >
                  {source.week} - {source.chapter_title}
                </a>
              </li>
            ))}
          </ul>
        </div>
      )}

      <div className="message-timestamp">
        {timeString}
      </div>
    </div>
  );
};