import React, { useState, useEffect } from 'react';
import { ChatAPIInstance } from './api';
import './PersonalizeButton.module.css';

interface PersonalizeContentButtonProps {
  chapterId: string;
  chapterContent: string;
  onContentUpdate: (content: string) => void;
  currentBackground?: string;
}

export const PersonalizeContentButton: React.FC<PersonalizeContentButtonProps> = ({
  chapterId,
  chapterContent,
  onContentUpdate,
  currentBackground
}) => {
  const [isProcessing, setIsProcessing] = useState(false);
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [originalContent, setOriginalContent] = useState(chapterContent);
  const [personalizedContent, setPersonalizedContent] = useState('');
  const [userBackground, setUserBackground] = useState(currentBackground || 'beginner');
  const [showBackgroundSelector, setShowBackgroundSelector] = useState(false);

  // Load personalized content from localStorage if it exists
  useEffect(() => {
    const savedPersonalizedContent = localStorage.getItem(`personalized_content_${chapterId}`);
    const savedBackground = localStorage.getItem(`personalized_background_${chapterId}`);
    const isPersonalizedFlag = localStorage.getItem(`is_personalized_${chapterId}`) === 'true';

    if (savedPersonalizedContent && isPersonalizedFlag) {
      setPersonalizedContent(savedPersonalizedContent);
      setIsPersonalized(true);
      if (savedBackground) {
        setUserBackground(savedBackground);
      }
      onContentUpdate(savedPersonalizedContent);
    }
  }, [chapterId, onContentUpdate]);

  const handlePersonalize = async () => {
    if (isProcessing) return;

    setIsProcessing(true);

    try {
      const token = localStorage.getItem('access_token');
      if (!token) {
        alert('Please log in to use personalization features');
        return;
      }

      const response = await ChatAPIInstance.personalize({
        chapter_id: chapterId,
        chapter_content: originalContent,
        background_level: userBackground
      });

      setPersonalizedContent(response.personalized_content);
      setIsPersonalized(true);

      // Save to localStorage for future use
      localStorage.setItem(`personalized_content_${chapterId}`, response.personalized_content);
      localStorage.setItem(`personalized_background_${chapterId}`, userBackground);
      localStorage.setItem(`is_personalized_${chapterId}`, 'true');

      onContentUpdate(response.personalized_content);
    } catch (error) {
      console.error('Error personalizing content:', error);
      alert('Failed to personalize content. Please try again.');
    } finally {
      setIsProcessing(false);
    }
  };

  const handleToggle = () => {
    if (isPersonalized) {
      // Switch back to original content
      onContentUpdate(originalContent);
      setIsPersonalized(false);
      localStorage.setItem(`is_personalized_${chapterId}`, 'false');
    } else {
      // Switch to personalized content
      if (personalizedContent) {
        onContentUpdate(personalizedContent);
        setIsPersonalized(true);
        localStorage.setItem(`is_personalized_${chapterId}`, 'true');
      } else {
        handlePersonalize(); // If no personalized content exists, generate it
      }
    }
  };

  const handleBackgroundChange = (newBackground: string) => {
    setUserBackground(newBackground);
    localStorage.setItem(`selected_background_${chapterId}`, newBackground);
    setShowBackgroundSelector(false);

    // If we already have personalized content for this background, use it
    // Otherwise, regenerate the content
    if (isPersonalized && personalizedContent) {
      // For simplicity, we'll regenerate when background changes
      setPersonalizedContent('');
      setIsPersonalized(false);
      localStorage.removeItem(`personalized_content_${chapterId}`);
      localStorage.removeItem(`is_personalized_${chapterId}`);
    }
  };

  const handleReset = () => {
    setPersonalizedContent('');
    setIsPersonalized(false);
    localStorage.removeItem(`personalized_content_${chapterId}`);
    localStorage.removeItem(`is_personalized_${chapterId}`);
    onContentUpdate(originalContent);
  };

  return (
    <div className="personalize-content-container">
      <div className="personalize-controls">
        <button
          className={`personalize-button ${isProcessing ? 'processing' : ''}`}
          onClick={handlePersonalize}
          disabled={isProcessing}
          title="Personalize content based on your background"
        >
          {isProcessing ? 'Processing...' : isPersonalized ? 'Re-personalize' : 'Personalize Content'}
        </button>

        {isPersonalized && (
          <button
            className="toggle-button"
            onClick={handleToggle}
            title={isPersonalized ? 'Show original content' : 'Show personalized content'}
          >
            {isPersonalized ? 'Show Original' : 'Show Personalized'}
          </button>
        )}

        <button
          className="background-selector-toggle"
          onClick={() => setShowBackgroundSelector(!showBackgroundSelector)}
          title="Change your background level"
        >
          Background: {userBackground}
        </button>

        {showBackgroundSelector && (
          <div className="background-selector-popup">
            <div className="background-option" onClick={() => handleBackgroundChange('beginner')}>
              Beginner
            </div>
            <div className="background-option" onClick={() => handleBackgroundChange('intermediate')}>
              Intermediate
            </div>
            <div className="background-option" onClick={() => handleBackgroundChange('advanced')}>
              Advanced
            </div>
          </div>
        )}

        {isPersonalized && (
          <button
            className="reset-button"
            onClick={handleReset}
            title="Reset personalization and start over"
          >
            Reset
          </button>
        )}
      </div>

      {isPersonalized && (
        <div className="personalization-info">
          Content personalized for: <strong>{userBackground}</strong> level
        </div>
      )}
    </div>
  );
};