// Client-side environment variables for Docusaurus
// This file is loaded by Docusaurus and makes environment variables available in the browser

// Define the API base URL - default to the backend running on port 8001
const API_BASE_URL =
  typeof window !== 'undefined'
    ? (window.ENV?.REACT_APP_API_BASE_URL || 'http://localhost:8001/api/v1')
    : (process.env.REACT_APP_API_BASE_URL || 'http://localhost:8001/api/v1');

// Make the environment variables available globally in the browser
if (typeof window !== 'undefined') {
  window.env = {
    REACT_APP_API_BASE_URL: API_BASE_URL,
  };
}

// Export for module usage
export const env = {
  REACT_APP_API_BASE_URL: API_BASE_URL,
};