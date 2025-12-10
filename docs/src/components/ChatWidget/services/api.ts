// Chat API service for the RAG Chatbot

interface ChatRequest {
  message: string;
  session_id: string;
  model?: string;
  conversation_context?: Array<{
    role: string;
    content: string;
  }>;
}

interface ChatResponse {
  response: string;
  sources: Array<{
    module: string;
    week: string;
    chapter_title: string;
    url: string;
  }>;
  conversation_id: string;
  message_id: string;
}

interface IngestRequest {
  module: string;
  week: string;
  chapter_title: string;
  content: string;
  url: string;
}

interface IngestResponse {
  status: string;
  chunk_id: string;
  token_count: number;
}

class ChatAPI {
  private baseUrl: string;
  private defaultHeaders: HeadersInit;

  constructor() {
    // Use environment variable or default to localhost
    this.baseUrl = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000/api/v1';

    this.defaultHeaders = {
      'Content-Type': 'application/json',
    };
  }

  async chat(request: ChatRequest): Promise<ChatResponse> {
    try {
      const response = await fetch(`${this.baseUrl}/chat`, {
        method: 'POST',
        headers: this.defaultHeaders,
        body: JSON.stringify(request),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error in chat API call:', error);
      throw error;
    }
  }

  async ingest(request: IngestRequest): Promise<IngestResponse> {
    try {
      const response = await fetch(`${this.baseUrl}/ingest`, {
        method: 'POST',
        headers: this.defaultHeaders,
        body: JSON.stringify(request),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error in ingest API call:', error);
      throw error;
    }
  }

  async healthCheck(): Promise<{ status: string; timestamp: string }> {
    try {
      const response = await fetch(`${this.baseUrl}/health`, {
        method: 'GET',
        headers: this.defaultHeaders,
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error in health check API call:', error);
      throw error;
    }
  }
}

// Export a singleton instance
export const ChatAPIInstance = new ChatAPI();

// Export the class for potential direct use
export { ChatAPI };