import React from 'react';
import { render, screen } from '@testing-library/react';
import '@testing-library/jest-dom';
import { MessageBubble } from '../MessageBubble';

describe('MessageBubble', () => {
  test('renders user message correctly', () => {
    render(
      <MessageBubble
        role="user"
        content="Test user message"
        timestamp={new Date('2023-01-01T10:00:00Z')}
        sources={[]}
      />
    );

    expect(screen.getByText('Test user message')).toBeInTheDocument();
  });

  test('renders assistant message correctly', () => {
    render(
      <MessageBubble
        role="assistant"
        content="Test assistant response"
        timestamp={new Date('2023-01-01T10:01:00Z')}
        sources={[
          {
            module: 'Test Module',
            week: 'Week 1',
            chapter_title: 'Test Chapter',
            url: '/docs/test-chapter'
          }
        ]}
      />
    );

    expect(screen.getByText('Test assistant response')).toBeInTheDocument();
  });

  test('displays sources for assistant messages', () => {
    render(
      <MessageBubble
        role="assistant"
        content="Test response with sources"
        timestamp={new Date('2023-01-01T10:02:00Z')}
        sources={[
          {
            module: 'Test Module',
            week: 'Week 1',
            chapter_title: 'Test Chapter',
            url: '/docs/test-chapter'
          },
          {
            module: 'Another Module',
            week: 'Week 2',
            chapter_title: 'Another Chapter',
            url: '/docs/another-chapter'
          }
        ]}
      />
    );

    expect(screen.getByText('Test Module')).toBeInTheDocument();
    expect(screen.getByText('Week 1')).toBeInTheDocument();
    expect(screen.getByText('Test Chapter')).toBeInTheDocument();

    expect(screen.getByText('Another Module')).toBeInTheDocument();
    expect(screen.getByText('Week 2')).toBeInTheDocument();
    expect(screen.getByText('Another Chapter')).toBeInTheDocument();

    // Check that both links are present
    const firstLink = screen.getByRole('link', { name: /Test Chapter/i });
    expect(firstLink).toBeInTheDocument();
    expect(firstLink).toHaveAttribute('href', '/docs/test-chapter');

    const secondLink = screen.getByRole('link', { name: /Another Chapter/i });
    expect(secondLink).toBeInTheDocument();
    expect(secondLink).toHaveAttribute('href', '/docs/another-chapter');
  });

  test('does not display sources for user messages', () => {
    render(
      <MessageBubble
        role="user"
        content="Test user message without sources"
        timestamp={new Date('2023-01-01T10:03:00Z')}
        sources={[
          {
            module: 'Test Module',
            week: 'Week 1',
            chapter_title: 'Test Chapter',
            url: '/docs/test-chapter'
          }
        ]}
      />
    );

    expect(screen.getByText('Test user message without sources')).toBeInTheDocument();

    // Sources should not be displayed for user messages
    expect(screen.queryByText('Test Module')).not.toBeInTheDocument();
    expect(screen.queryByText('Week 1')).not.toBeInTheDocument();
    expect(screen.queryByText('Test Chapter')).not.toBeInTheDocument();
  });

  test('renders correctly with empty sources array', () => {
    render(
      <MessageBubble
        role="assistant"
        content="Test response with no sources"
        timestamp={new Date('2023-01-01T10:04:00Z')}
        sources={[]}
      />
    );

    expect(screen.getByText('Test response with no sources')).toBeInTheDocument();
    // No source elements should be present
    expect(screen.queryByText('Sources:')).not.toBeInTheDocument();
  });

  test('formats timestamp correctly', () => {
    render(
      <MessageBubble
        role="user"
        content="Test message with timestamp"
        timestamp={new Date('2023-01-01T10:05:00Z')}
        sources={[]}
      />
    );

    expect(screen.getByText('Test message with timestamp')).toBeInTheDocument();
  });

  test('handles message with special characters', () => {
    render(
      <MessageBubble
        role="assistant"
        content="Test message with <special> & characters"
        timestamp={new Date('2023-01-01T10:06:00Z')}
        sources={[
          {
            module: 'Special Module',
            week: 'Week X',
            chapter_title: 'Special Chapter: <Tag> & More',
            url: '/docs/special-chapter'
          }
        ]}
      />
    );

    expect(screen.getByText('Test message with <special> & characters')).toBeInTheDocument();
    expect(screen.getByText('Special Module')).toBeInTheDocument();
    expect(screen.getByText('Week X')).toBeInTheDocument();
    expect(screen.getByText('Special Chapter: <Tag> & More')).toBeInTheDocument();
  });

  test('handles message with very long content', () => {
    const longContent = 'A'.repeat(1000); // Long message content
    render(
      <MessageBubble
        role="user"
        content={longContent}
        timestamp={new Date('2023-01-01T10:07:00Z')}
        sources={[]}
      />
    );

    expect(screen.getByText(longContent)).toBeInTheDocument();
  });

  test('handles typing indicator', () => {
    render(
      <MessageBubble
        role="assistant"
        content=""
        timestamp={new Date('2023-01-01T10:08:00Z')}
        isTyping={true}
      />
    );

    expect(screen.getByText('')).toBeInTheDocument();
    expect(screen.getByTestId('typing-indicator')).toBeInTheDocument();
  });
});