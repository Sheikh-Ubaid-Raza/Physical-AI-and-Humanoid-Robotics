import React, { useState, useEffect } from 'react';
import { ChatAPIInstance } from './api';
import './TranslationButton.module.css';

interface TranslationButtonProps {
  chapterId: string;
  chapterContent: string;
  onContentUpdate: (content: string, language?: string) => void;
  currentLanguage?: string;
}

export const TranslationButton: React.FC<TranslationButtonProps> = ({
  chapterId,
  chapterContent,
  onContentUpdate,
  currentLanguage = 'en'
}) => {
  const [isProcessing, setIsProcessing] = useState(false);
  const [isTranslated, setIsTranslated] = useState(false);
  const [originalContent, setOriginalContent] = useState(chapterContent);
  const [translatedContent, setTranslatedContent] = useState('');
  const [targetLanguage, setTargetLanguage] = useState('ur'); // Default to Urdu
  const [showLanguageSelector, setShowLanguageSelector] = useState(false);

  // Load translated content from localStorage if it exists
  useEffect(() => {
    const savedTranslatedContent = localStorage.getItem(`translated_content_${chapterId}_${targetLanguage}`);
    const isTranslatedFlag = localStorage.getItem(`is_translated_${chapterId}_${targetLanguage}`) === 'true';

    if (savedTranslatedContent && isTranslatedFlag) {
      setTranslatedContent(savedTranslatedContent);
      setIsTranslated(true);
      onContentUpdate(savedTranslatedContent, targetLanguage);
    }
  }, [chapterId, targetLanguage, onContentUpdate]);

  const handleTranslate = async () => {
    if (isProcessing) return;

    setIsProcessing(true);

    try {
      const token = localStorage.getItem('access_token');
      if (!token) {
        alert('Please log in to use translation features');
        return;
      }

      const response = await ChatAPIInstance.translate({
        chapter_id: chapterId,
        chapter_content: originalContent,
        target_language: targetLanguage
      });

      setTranslatedContent(response.translated_content);
      setIsTranslated(true);

      // Save to localStorage for future use
      localStorage.setItem(`translated_content_${chapterId}_${targetLanguage}`, response.translated_content);
      localStorage.setItem(`is_translated_${chapterId}_${targetLanguage}`, 'true');

      onContentUpdate(response.translated_content, targetLanguage);
    } catch (error) {
      console.error('Error translating content:', error);
      alert('Failed to translate content. Please try again.');
    } finally {
      setIsProcessing(false);
    }
  };

  const handleToggle = () => {
    if (isTranslated) {
      // Switch back to original content
      onContentUpdate(originalContent, 'en');
      setIsTranslated(false);
      localStorage.setItem(`is_translated_${chapterId}_${targetLanguage}`, 'false');
    } else {
      // Switch to translated content
      if (translatedContent) {
        onContentUpdate(translatedContent, targetLanguage);
        setIsTranslated(true);
        localStorage.setItem(`is_translated_${chapterId}_${targetLanguage}`, 'true');
      } else {
        handleTranslate(); // If no translated content exists, translate it
      }
    }
  };

  const handleLanguageChange = (newLanguage: string) => {
    setTargetLanguage(newLanguage);
    setShowLanguageSelector(false);

    // If we already have translated content for this language, use it
    // Otherwise, translate the content
    if (isTranslated && translatedContent) {
      // For simplicity, we'll retranslate when language changes
      setTranslatedContent('');
      setIsTranslated(false);
      localStorage.removeItem(`translated_content_${chapterId}_${newLanguage}`);
      localStorage.removeItem(`is_translated_${chapterId}_${newLanguage}`);
    }
  };

  const handleReset = () => {
    setTranslatedContent('');
    setIsTranslated(false);
    localStorage.removeItem(`translated_content_${chapterId}_${targetLanguage}`);
    localStorage.removeItem(`is_translated_${chapterId}_${targetLanguage}`);
    onContentUpdate(originalContent, 'en');
  };

  return (
    <div className="translation-container">
      <div className="translation-controls">
        <button
          className={`translate-button ${isProcessing ? 'processing' : ''}`}
          onClick={handleTranslate}
          disabled={isProcessing}
          title="Translate content to selected language"
        >
          {isProcessing ? 'Translating...' : isTranslated ? 'Re-translate' : `Translate to ${targetLanguage === 'ur' ? 'Urdu' : targetLanguage.toUpperCase()}`}
        </button>

        {isTranslated && (
          <button
            className="toggle-language-button"
            onClick={handleToggle}
            title={isTranslated ? 'Show original content' : 'Show translated content'}
          >
            {isTranslated ? 'Show Original' : 'Show Translation'}
          </button>
        )}

        <button
          className="language-selector-toggle"
          onClick={() => setShowLanguageSelector(!showLanguageSelector)}
          title="Change target language"
        >
          {targetLanguage === 'ur' ? ' Urdu ' : targetLanguage.toUpperCase()}
        </button>

        {showLanguageSelector && (
          <div className="language-selector-popup">
            <div className="language-option" onClick={() => handleLanguageChange('ur')}>
              Urdu
            </div>
            <div className="language-option" onClick={() => handleLanguageChange('es')}>
              Spanish
            </div>
            <div className="language-option" onClick={() => handleLanguageChange('fr')}>
              French
            </div>
            <div className="language-option" onClick={() => handleLanguageChange('de')}>
              German
            </div>
          </div>
        )}

        {isTranslated && (
          <button
            className="reset-translation-button"
            onClick={handleReset}
            title="Reset translation and start over"
          >
            Reset
          </button>
        )}
      </div>

      {isTranslated && (
        <div className={`translation-info ${targetLanguage === 'ur' ? 'rtl-language' : ''}`}>
          Content translated to: <strong>{targetLanguage === 'ur' ? 'Urdu' : targetLanguage.toUpperCase()}</strong>
        </div>
      )}

      {isTranslated && targetLanguage === 'ur' && (
        <div className="urdu-content-wrapper">
          {/* This will be styled with RTL in CSS */}
        </div>
      )}
    </div>
  );
};