import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import { ChatWidget } from '../ChatWidget';

// Mock the ChatModal component to avoid complex integration testing
jest.mock('../ChatModal', () => ({
  ChatModal: ({ isOpen, onClose }: { isOpen: boolean; onClose: () => void }) => (
    isOpen ? (
      <div role="dialog">
        <input placeholder="Ask a question about the textbook..." />
        <button onClick={onClose}>Close</button>
        <button>Send</button>
      </div>
    ) : null
  )
}));

// Mock localStorage
const localStorageMock = {
  getItem: jest.fn(),
  setItem: jest.fn(),
  removeItem: jest.fn(),
  clear: jest.fn(),
};
Object.defineProperty(window, 'localStorage', {
  value: localStorageMock,
});

describe('ChatWidget', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    localStorageMock.getItem.mockReturnValue(null); // No stored session initially
  });

  test('renders chat widget button initially', () => {
    render(<ChatWidget />);

    const chatButton = screen.getByRole('button');
    expect(chatButton).toBeInTheDocument();
    expect(chatButton).toHaveTextContent('💬');
  });

  test('opens chat modal when button is clicked', () => {
    render(<ChatWidget />);

    const chatButton = screen.getByRole('button');
    fireEvent.click(chatButton);

    expect(screen.getByRole('dialog')).toBeInTheDocument();
  });

  test('closes chat modal when close button is clicked', () => {
    render(<ChatWidget />);

    // Open the modal first
    const chatButton = screen.getByRole('button');
    fireEvent.click(chatButton);

    // Find and click the close button
    const closeButton = screen.getByRole('button', { name: /Close/i });
    fireEvent.click(closeButton);

    expect(screen.queryByRole('dialog')).not.toBeInTheDocument();
  });

  test('generates session ID if none exists', () => {
    localStorageMock.getItem.mockReturnValue(null);

    render(<ChatWidget />);

    expect(localStorageMock.setItem).toHaveBeenCalledWith(
      'chatbot_session_id',
      expect.any(String)
    );
  });

  test('uses existing session ID if available', () => {
    const existingSessionId = 'existing-session-id';
    localStorageMock.getItem.mockReturnValueOnce(existingSessionId);

    render(<ChatWidget />);

    expect(localStorageMock.getItem).toHaveBeenCalledWith('chatbot_session_id');
  });
});