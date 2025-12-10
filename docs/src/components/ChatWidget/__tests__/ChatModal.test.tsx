import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import { ChatModal } from '../ChatModal';

// Mock the API service
jest.mock('../services/api', () => ({
  chatService: {
    sendMessage: jest.fn().mockResolvedValue({
      response: 'Test response from API',
      sources: [
        {
          module: 'Test Module',
          week: 'Week 1',
          chapter_title: 'Test Chapter',
          url: '/docs/test-chapter',
          relevance_score: 0.85
        }
      ],
      conversation_id: 'test-conversation-id',
      message_id: 'test-message-id'
    }),
    createSession: jest.fn().mockReturnValue('test-session-id')
  }
}));

describe('ChatModal', () => {
  const mockOnClose = jest.fn();
  const mockOnSendMessage = jest.fn();
  const mockMessages: any[] = [];
  const mockMessagesEndRef = { current: null };

  beforeEach(() => {
    jest.clearAllMocks();
  });

  test('renders chat modal with input field and close button', () => {
    render(
      <ChatModal
        isOpen={true}
        onClose={mockOnClose}
        onSendMessage={mockOnSendMessage}
        messages={mockMessages}
        isLoading={false}
        onClear={jest.fn()}
        messagesEndRef={mockMessagesEndRef}
      />
    );

    expect(screen.getByRole('dialog')).toBeInTheDocument();
    expect(screen.getByPlaceholderText('Ask a question about the textbook...')).toBeInTheDocument();
    expect(screen.getByRole('button', { name: /close/i })).toBeInTheDocument();
  });

  test('calls onClose when close button is clicked', () => {
    render(
      <ChatModal
        isOpen={true}
        onClose={mockOnClose}
        onSendMessage={mockOnSendMessage}
        messages={mockMessages}
        isLoading={false}
        onClear={jest.fn()}
        messagesEndRef={mockMessagesEndRef}
      />
    );

    const closeButton = screen.getByRole('button', { name: /close/i });
    fireEvent.click(closeButton);

    expect(mockOnClose).toHaveBeenCalledTimes(1);
  });

  test('allows user to type and submit a message', async () => {
    render(
      <ChatModal
        isOpen={true}
        onClose={mockOnClose}
        onSendMessage={mockOnSendMessage}
        messages={mockMessages}
        isLoading={false}
        onClear={jest.fn()}
        messagesEndRef={mockMessagesEndRef}
      />
    );

    const input = screen.getByPlaceholderText('Ask a question about the textbook...');
    fireEvent.change(input, { target: { value: 'Test message' } });

    const sendButton = screen.getByRole('button', { name: /send/i });
    fireEvent.click(sendButton);

    await waitFor(() => {
      expect(mockOnSendMessage).toHaveBeenCalledWith('Test message');
    });
  });

  test('submits message when Enter key is pressed', async () => {
    render(
      <ChatModal
        isOpen={true}
        onClose={mockOnClose}
        onSendMessage={mockOnSendMessage}
        messages={mockMessages}
        isLoading={false}
        onClear={jest.fn()}
        messagesEndRef={mockMessagesEndRef}
      />
    );

    const input = screen.getByPlaceholderText('Ask a question about the textbook...');
    fireEvent.change(input, { target: { value: 'Test message with Enter' } });
    fireEvent.keyDown(input, { key: 'Enter', code: 'Enter' });

    await waitFor(() => {
      expect(mockOnSendMessage).toHaveBeenCalledWith('Test message with Enter');
    });
  });

  test('prevents submission of empty messages', () => {
    render(
      <ChatModal
        isOpen={true}
        onClose={mockOnClose}
        onSendMessage={mockOnSendMessage}
        messages={mockMessages}
        isLoading={false}
        onClear={jest.fn()}
        messagesEndRef={mockMessagesEndRef}
      />
    );

    const input = screen.getByPlaceholderText('Ask a question about the textbook...');
    fireEvent.change(input, { target: { value: '' } });

    const sendButton = screen.getByRole('button', { name: /send/i });
    fireEvent.click(sendButton);

    expect(mockOnSendMessage).not.toHaveBeenCalled();
  });

  test('disables input when loading', () => {
    render(
      <ChatModal
        isOpen={true}
        onClose={mockOnClose}
        onSendMessage={mockOnSendMessage}
        messages={mockMessages}
        isLoading={true}
        onClear={jest.fn()}
        messagesEndRef={mockMessagesEndRef}
      />
    );

    const input = screen.getByPlaceholderText('Ask a question about the textbook...');
    expect(input).toBeDisabled();
  });
});