import React, { useState, useEffect, useRef } from 'react';
import { ChatAPIInstance } from './api';
import './TextSelectionTooltip.module.css';

interface TextSelectionTooltipProps {
  children: React.ReactNode;
}

interface SelectedTextData {
  text: string;
  rect: DOMRect;
}

export const TextSelectionTooltip: React.FC<TextSelectionTooltipProps> = ({ children }) => {
  const [selectedTextData, setSelectedTextData] = useState<SelectedTextData | null>(null);
  const [showTooltip, setShowTooltip] = useState(false);
  const [tooltipPosition, setTooltipPosition] = useState({ x: 0, y: 0 });
  const [isLoading, setIsLoading] = useState(false);
  const [response, setResponse] = useState<string | null>(null);
  const tooltipRef = useRef<HTMLDivElement>(null);

  // Handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim() !== '') {
        const selectedText = selection.toString().trim();
        if (selectedText.length > 0 && selectedText.length <= 5000) { // Match backend limit
          const range = selection.getRangeAt(0);
          const rect = range.getBoundingClientRect();

          // Position tooltip above the selected text
          setTooltipPosition({
            x: rect.left + window.scrollX,
            y: rect.top + window.scrollY - 40 // 40px above the selection
          });

          setSelectedTextData({
            text: selectedText,
            rect: rect
          });
          setShowTooltip(true);
        }
      } else {
        setShowTooltip(false);
        setResponse(null);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  // Handle keyboard shortcut (Ctrl/Cmd + Enter)
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if ((e.ctrlKey || e.metaKey) && e.key === 'Enter' && selectedTextData) {
        handleAskAI();
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, [selectedTextData]);

  // Close tooltip when clicking elsewhere
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (tooltipRef.current && !tooltipRef.current.contains(event.target as Node)) {
        setShowTooltip(false);
        setResponse(null);
      }
    };

    if (showTooltip) {
      document.addEventListener('mousedown', handleClickOutside);
    }
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [showTooltip]);

  const handleAskAI = async () => {
    if (!selectedTextData) return;

    setIsLoading(true);
    setResponse(null);

    try {
      // Use the existing session ID from localStorage
      const sessionId = localStorage.getItem('chatbot_session_id') || 'default_session';

      const result = await ChatAPIInstance.selectedText({
        message: "Explain this selected text and provide additional context.",
        session_id: sessionId,
        selected_text: selectedTextData.text
      });

      setResponse(result.response);
    } catch (error) {
      console.error('Error getting AI response for selected text:', error);
      setResponse('Sorry, I encountered an error processing your selected text. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      {children}
      {showTooltip && (
        <div
          ref={tooltipRef}
          className="text-selection-tooltip"
          style={{
            position: 'absolute',
            left: `${tooltipPosition.x}px`,
            top: `${tooltipPosition.y}px`,
            zIndex: 10000
          }}
        >
          <button
            className="tooltip-button"
            onClick={handleAskAI}
            disabled={isLoading}
            title="Ask AI about selected text (Ctrl/Cmd+Enter)"
          >
            {isLoading ? 'Processing...' : 'Ask AI'}
          </button>
          {response && (
            <div className="tooltip-response">
              <div className="response-content">{response}</div>
            </div>
          )}
        </div>
      )}
    </>
  );
};