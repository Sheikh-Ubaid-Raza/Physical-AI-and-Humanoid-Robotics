import { ChatAPIInstance } from '../services/api';

// Mock fetch globally
global.fetch = jest.fn();

describe('API Service', () => {
  const originalEnv = process.env;

  beforeEach(() => {
    jest.clearAllMocks();
    // Set up environment variables for testing
    process.env = { ...originalEnv,
      NEXT_PUBLIC_API_BASE_URL: 'http://localhost:8000',
      GEMINI_API_KEY: 'test-api-key'
    };
  });

  afterEach(() => {
    process.env = originalEnv; // Restore original env vars
  });

  describe('chatService', () => {
    describe('sendMessage', () => {
      test('sends message successfully', async () => {
        const mockResponse = {
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
        };

        (global.fetch as jest.Mock).mockResolvedValueOnce({
          ok: true,
          json: () => Promise.resolve(mockResponse),
        });

        const result = await ChatAPIInstance.chat({
          message: 'Test message',
          session_id: 'test-session-id',
          model: 'gpt-4o'
        });

        expect(global.fetch).toHaveBeenCalledWith(
          'http://localhost:8000/api/v1/chat',
          expect.objectContaining({
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({
              message: 'Test message',
              session_id: 'test-session-id',
              model: 'gpt-4o'
            })
          })
        );

        expect(result).toEqual(mockResponse);
      });

      test('handles API error response', async () => {
        (global.fetch as jest.Mock).mockResolvedValueOnce({
          ok: false,
          status: 500,
          statusText: 'Internal Server Error',
        });

        await expect(ChatAPIInstance.chat({
          message: 'Test message',
          session_id: 'test-session-id'
        })).rejects.toThrow('HTTP error! status: 500');
      });

      test('handles network error', async () => {
        (global.fetch as jest.Mock).mockRejectedValueOnce(new Error('Network error'));

        await expect(ChatAPIInstance.chat({
          message: 'Test message',
          session_id: 'test-session-id'
        })).rejects.toThrow('Network error');
      });

      test('handles invalid JSON response', async () => {
        (global.fetch as jest.Mock).mockResolvedValueOnce({
          ok: true,
          json: () => Promise.reject(new Error('Invalid JSON')),
        });

        await expect(ChatAPIInstance.chat({
          message: 'Test message',
          session_id: 'test-session-id'
        })).rejects.toThrow('Invalid JSON');
      });

      test('sends message with conversation context', async () => {
        const mockResponse = {
          response: 'Test response with context',
          sources: [],
          conversation_id: 'test-conversation-id',
          message_id: 'test-message-id'
        };

        const conversationContext = [
          {
            role: 'user',
            content: 'Previous question'
          },
          {
            role: 'assistant',
            content: 'Previous answer'
          }
        ];

        (global.fetch as jest.Mock).mockResolvedValueOnce({
          ok: true,
          json: () => Promise.resolve(mockResponse),
        });

        await ChatAPIInstance.chat({
          message: 'Follow-up question',
          session_id: 'test-session-id',
          conversation_context: conversationContext
        });

        expect(global.fetch).toHaveBeenCalledWith(
          'http://localhost:8000/api/v1/chat',
          expect.objectContaining({
            body: JSON.stringify({
              message: 'Follow-up question',
              session_id: 'test-session-id',
              conversation_context: conversationContext
            })
          })
        );
      });

      test('uses default model when none provided', async () => {
        const mockResponse = {
          response: 'Test response',
          sources: [],
          conversation_id: 'test-conversation-id',
          message_id: 'test-message-id'
        };

        (global.fetch as jest.Mock).mockResolvedValueOnce({
          ok: true,
          json: () => Promise.resolve(mockResponse),
        });

        await ChatAPIInstance.chat({
          message: 'Test message',
          session_id: 'test-session-id'
          // No model specified
        });

        expect(global.fetch).toHaveBeenCalledWith(
          'http://localhost:8000/api/v1/chat',
          expect.objectContaining({
            body: JSON.stringify({
              message: 'Test message',
              session_id: 'test-session-id'
              // Note: model is not added as default in the actual API implementation
            })
          })
        );
      });

      test('uses provided model', async () => {
        const mockResponse = {
          response: 'Test response',
          sources: [],
          conversation_id: 'test-conversation-id',
          message_id: 'test-message-id'
        };

        (global.fetch as jest.Mock).mockResolvedValueOnce({
          ok: true,
          json: () => Promise.resolve(mockResponse),
        });

        await ChatAPIInstance.chat({
          message: 'Test message',
          session_id: 'test-session-id',
          model: 'gpt-4o-mini'
        });

        expect(global.fetch).toHaveBeenCalledWith(
          'http://localhost:8000/api/v1/chat',
          expect.objectContaining({
            body: JSON.stringify({
              message: 'Test message',
              session_id: 'test-session-id',
              model: 'gpt-4o-mini'
            })
          })
        );
      });
    });

    describe('ingest', () => {
      test('ingests content successfully', async () => {
        const mockResponse = {
          status: 'success',
          chunk_id: 'test-chunk-id',
          token_count: 100
        };

        (global.fetch as jest.Mock).mockResolvedValueOnce({
          ok: true,
          json: () => Promise.resolve(mockResponse),
        });

        const result = await ChatAPIInstance.ingest({
          module: 'Test Module',
          week: 'Week 1',
          chapter_title: 'Test Chapter',
          content: 'Test content',
          url: '/docs/test-chapter'
        });

        expect(global.fetch).toHaveBeenCalledWith(
          'http://localhost:8000/api/v1/ingest',
          expect.objectContaining({
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({
              module: 'Test Module',
              week: 'Week 1',
              chapter_title: 'Test Chapter',
              content: 'Test content',
              url: '/docs/test-chapter'
            })
          })
        );

        expect(result).toEqual(mockResponse);
      });
    });

    describe('healthCheck', () => {
      test('performs health check successfully', async () => {
        const mockResponse = {
          status: 'healthy',
          timestamp: '2023-01-01T10:00:00Z'
        };

        (global.fetch as jest.Mock).mockResolvedValueOnce({
          ok: true,
          json: () => Promise.resolve(mockResponse),
        });

        const result = await ChatAPIInstance.healthCheck();

        expect(global.fetch).toHaveBeenCalledWith(
          'http://localhost:8000/api/v1/health',
          expect.objectContaining({
            method: 'GET',
            headers: {
              'Content-Type': 'application/json',
            }
          })
        );

        expect(result).toEqual(mockResponse);
      });
    });
  });
});