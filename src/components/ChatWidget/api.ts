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

// Authentication interfaces
interface UserRegisterRequest {
  name: string;
  email: string;
  password: string;
  software_background: string;  // beginner, intermediate, advanced
  hardware_background: string;  // none, basic, advanced
}

interface UserLoginRequest {
  email: string;
  password: string;
}

interface UserResponse {
  id: string;
  name: string;
  email: string;
  software_background: string;
  hardware_background: string;
}

interface LoginResponse {
  access_token: string;
  token_type: string;
  user: UserResponse;
}

interface CurrentUserResponse {
  id: string;
  name: string;
  email: string;
  software_background: string;
  hardware_background: string;
}

// Selected text feature interfaces
interface SelectedTextRequest {
  message: string;
  session_id: string;
  selected_text: string;
  conversation_context?: Array<{
    role: string;
    content: string;
  }>;
  model?: string;
}

interface SelectedTextResponse {
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

// Personalization feature interfaces
interface PersonalizeRequest {
  chapter_id: string;
  chapter_content: string;
  background_level?: string;  // Optional override for user's background level
}

interface PersonalizeResponse {
  personalized_content: string;
  original_background: string;
  processing_time: number;
}

// Translation feature interfaces
interface TranslateRequest {
  chapter_id: string;
  chapter_content: string;
  target_language?: string;  // Default to 'ur' (Urdu)
}

interface TranslateResponse {
  translated_content: string;
  original_language: string;
  target_language: string;
  processing_time: number;
  preserved_elements: Record<string, any>;  // Metadata about preserved elements
}

class ChatAPI {
  private baseUrl: string;
  private defaultHeaders: HeadersInit;

  constructor() {
    // Use environment variable or default to localhost
    // Check if we're in a Node.js environment (process exists) or browser environment
    let apiUrl = 'http://localhost:8001/api/v1'; // Updated to match our running backend

    if (typeof process !== 'undefined' && process.env) {
      // Node.js environment
      apiUrl = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8001/api/v1';
    } else if (typeof window !== 'undefined' && (window as any).env) {
      // Browser environment - check for window.env (set by Docusaurus config)
      apiUrl = (window as any).env.REACT_APP_API_BASE_URL || 'http://localhost:8001/api/v1';
    } else {
      // Browser environment - fallback
      // For GitHub Pages deployment, we can use relative path or the deployed backend URL
      apiUrl = 'http://localhost:8001/api/v1'; // Updated to match our running backend
    }

    this.baseUrl = apiUrl;

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

  // Authentication methods
  async register(request: UserRegisterRequest): Promise<UserResponse> {
    try {
      const response = await fetch(`${this.baseUrl}/auth/signup`, {
        method: 'POST',
        headers: this.defaultHeaders,
        body: JSON.stringify(request),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error in register API call:', error);
      throw error;
    }
  }

  async login(request: UserLoginRequest): Promise<LoginResponse> {
    try {
      const response = await fetch(`${this.baseUrl}/auth/signin`, {
        method: 'POST',
        headers: this.defaultHeaders,
        body: JSON.stringify(request),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error in login API call:', error);
      throw error;
    }
  }

  async logout(): Promise<{ message: string }> {
    try {
      const response = await fetch(`${this.baseUrl}/auth/signout`, {
        method: 'POST',
        headers: this.defaultHeaders,
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error in logout API call:', error);
      throw error;
    }
  }

  async getCurrentUser(): Promise<CurrentUserResponse> {
    try {
      // Get token from localStorage or wherever it's stored
      const token = localStorage.getItem('access_token');
      if (!token) {
        throw new Error('No access token found');
      }

      const response = await fetch(`${this.baseUrl}/auth/me`, {
        method: 'GET',
        headers: {
          ...this.defaultHeaders,
          'Authorization': `Bearer ${token}`
        },
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error in getCurrentUser API call:', error);
      throw error;
    }
  }

  // Selected text feature method
  async selectedText(request: SelectedTextRequest): Promise<SelectedTextResponse> {
    try {
      const response = await fetch(`${this.baseUrl}/chat/selection`, {
        method: 'POST',
        headers: this.defaultHeaders,
        body: JSON.stringify(request),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error in selectedText API call:', error);
      throw error;
    }
  }

  // Personalization feature method
  async personalize(request: PersonalizeRequest): Promise<PersonalizeResponse> {
    try {
      // Get token from localStorage or wherever it's stored
      const token = localStorage.getItem('access_token');
      if (!token) {
        throw new Error('No access token found');
      }

      const response = await fetch(`${this.baseUrl}/personalize`, {
        method: 'POST',
        headers: {
          ...this.defaultHeaders,
          'Authorization': `Bearer ${token}`
        },
        body: JSON.stringify(request),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error in personalize API call:', error);
      throw error;
    }
  }

  // Translation feature method
  async translate(request: TranslateRequest): Promise<TranslateResponse> {
    try {
      // Get token from localStorage or wherever it's stored
      const token = localStorage.getItem('access_token');
      if (!token) {
        throw new Error('No access token found');
      }

      const response = await fetch(`${this.baseUrl}/translate`, {
        method: 'POST',
        headers: {
          ...this.defaultHeaders,
          'Authorization': `Bearer ${token}`
        },
        body: JSON.stringify(request),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Error in translate API call:', error);
      throw error;
    }
  }
}

// Export a singleton instance
export const ChatAPIInstance = new ChatAPI();

// Export the class for potential direct use
export { ChatAPI };