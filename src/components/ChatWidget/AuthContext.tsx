import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { ChatAPIInstance } from './api';

interface User {
  id: string;
  name: string;
  email: string;
  software_background: string;
  hardware_background: string;
}

interface AuthContextType {
  user: User | null;
  loading: boolean;
  login: (email: string, password: string) => Promise<boolean>;
  logout: () => void;
  register: (userData: {
    name: string;
    email: string;
    password: string;
    software_background: string;
    hardware_background: string;
  }) => Promise<boolean>;
  isAuthenticated: () => boolean;
  refreshUser: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // Check if user is logged in on initial load
    const checkAuthStatus = async () => {
      try {
        const token = localStorage.getItem('access_token');
        if (token) {
          await refreshUser();
        }
      } catch (error) {
        console.error('Auth check failed:', error);
        // Clear invalid token
        localStorage.removeItem('access_token');
      } finally {
        setLoading(false);
      }
    };

    checkAuthStatus();
  }, []);

  const refreshUser = async () => {
    try {
      const userData = await ChatAPIInstance.getCurrentUser();
      setUser(userData);
    } catch (error) {
      console.error('Failed to refresh user:', error);
      // Clear invalid token
      localStorage.removeItem('access_token');
      setUser(null);
      throw error;
    }
  };

  const login = async (email: string, password: string): Promise<boolean> => {
    try {
      setLoading(true);
      const response = await ChatAPIInstance.login({ email, password });

      // Store token in localStorage
      localStorage.setItem('access_token', response.access_token);

      // Set user data
      setUser(response.user);

      return true;
    } catch (error) {
      console.error('Login failed:', error);
      return false;
    } finally {
      setLoading(false);
    }
  };

  const register = async (userData: {
    name: string;
    email: string;
    password: string;
    software_background: string;
    hardware_background: string;
  }): Promise<boolean> => {
    try {
      setLoading(true);
      const response = await ChatAPIInstance.register(userData);

      // Auto-login after successful registration
      const loginSuccess = await login(userData.email, userData.password);

      return loginSuccess;
    } catch (error) {
      console.error('Registration failed:', error);
      return false;
    } finally {
      setLoading(false);
    }
  };

  const logout = () => {
    localStorage.removeItem('access_token');
    setUser(null);
  };

  const isAuthenticated = (): boolean => {
    return !!user;
  };

  const value: AuthContextType = {
    user,
    loading,
    login,
    logout,
    register,
    isAuthenticated,
    refreshUser
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

export const useAuth = (): AuthContextType => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};