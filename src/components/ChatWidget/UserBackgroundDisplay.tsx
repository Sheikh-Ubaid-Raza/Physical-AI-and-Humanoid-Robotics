import React, { useState, useEffect } from 'react';
import { ChatAPIInstance } from './api';
import './UserBackgroundDisplay.module.css';

interface UserBackgroundDisplayProps {
  onBackgroundChange?: (background: { software: string; hardware: string }) => void;
}

export const UserBackgroundDisplay: React.FC<UserBackgroundDisplayProps> = ({ onBackgroundChange }) => {
  const [user, setUser] = useState<any>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    loadCurrentUser();
  }, []);

  const loadCurrentUser = async () => {
    try {
      setLoading(true);
      const userData = await ChatAPIInstance.getCurrentUser();
      setUser(userData);

      if (onBackgroundChange) {
        onBackgroundChange({
          software: userData.software_background,
          hardware: userData.hardware_background
        });
      }
    } catch (err) {
      setError('Not logged in');
      console.error('Error loading user:', err);
    } finally {
      setLoading(false);
    }
  };

  if (loading) {
    return <div className="user-background-display">Loading user info...</div>;
  }

  if (error || !user) {
    return (
      <div className="user-background-display user-not-logged-in">
        <span className="not-logged-in-text">Not logged in</span>
        <span className="background-placeholder">Software: -- | Hardware: --</span>
      </div>
    );
  }

  return (
    <div className="user-background-display">
      <div className="user-info">
        <span className="user-name">{user.name}</span>
      </div>
      <div className="background-info">
        <span className="background-item">
          <span className="label">Software:</span>
          <span className="value">{user.software_background}</span>
        </span>
        <span className="background-separator">|</span>
        <span className="background-item">
          <span className="label">Hardware:</span>
          <span className="value">{user.hardware_background}</span>
        </span>
      </div>
    </div>
  );
};