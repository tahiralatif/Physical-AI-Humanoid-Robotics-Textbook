import React, { useState, useEffect } from 'react';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import { ThemeClassNames } from '@docusaurus/theme-common';
import { useLocation } from '@docusaurus/router';
import clsx from 'clsx';
import useIsBrowser from '@docusaurus/useIsBrowser';

// Utility function to check if user is authenticated
const useAuth = () => {
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const isBrowser = useIsBrowser();

  useEffect(() => {
    if (isBrowser) {
      const token = localStorage.getItem('better-auth.session_token');
      setIsAuthenticated(!!token);
    }
  }, [isBrowser]);

  return { isAuthenticated };
};

// Function to update navigation elements based on language preference
const updateNavigationForLanguage = (languagePref) => {
  if (languagePref === 'ur') {
    // Translate navigation elements to Urdu
    const navItems = document.querySelectorAll('.navbar__item, .menu__link, .sidebar__item');
    navItems.forEach(item => {
      // Add a temporary class for Urdu navigation items to identify them for styling
      item.classList.add('urdu-nav-item');
    });
  } else {
    // Remove Urdu navigation class when switching back to English
    const navItems = document.querySelectorAll('.urdu-nav-item');
    navItems.forEach(item => {
      item.classList.remove('urdu-nav-item');
    });
  }
};

// Component to show Personalize for Me button
const PersonalizeButton = ({ chapterId, metadata }) => {
  const [isLoading, setIsLoading] = useState(false);
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [personalizedContent, setPersonalizedContent] = useState(null);
  const [showOriginal, setShowOriginal] = useState(true);
  const { isAuthenticated } = useAuth();

  const handlePersonalize = async () => {
    if (!isAuthenticated) {
      alert('Please sign in to personalize content');
      return;
    }

    setIsLoading(true);
    try {
      // Get the user's profile to determine expertise level
      const profileResponse = await fetch('/api/user/profile', {
        method: 'GET',
        credentials: 'include',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!profileResponse.ok) {
        throw new Error('Failed to fetch user profile');
      }

      const userProfile = await profileResponse.json();

      // Call the personalization API
      const personalizeResponse = await fetch(`/api/personalize/chapter/${chapterId}`, {
        method: 'POST',
        credentials: 'include',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapterId,
          userProfile,
          metadata
        }),
      });

      if (!personalizeResponse.ok) {
        throw new Error('Failed to personalize content');
      }

      const data = await personalizeResponse.json();
      setPersonalizedContent(data.personalizedContent);
      setIsPersonalized(true);
      setShowOriginal(false);
    } catch (error) {
      console.error('Error personalizing content:', error);
      alert('Failed to personalize content. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  if (!isAuthenticated) {
    return null; // Don't show button if not authenticated
  }

  return (
    <div className="personalize-section" style={{ margin: '20px 0', padding: '15px', border: '1px solid #e0e0e0', borderRadius: '8px', backgroundColor: '#f9f9f9' }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '10px' }}>
        <h3 style={{ margin: 0, fontSize: '1.2em' }}>Personalize this chapter</h3>
        <button
          onClick={handlePersonalize}
          disabled={isLoading || isPersonalized}
          className={clsx('button button--primary', { 'button--loading': isLoading })}
          style={{ padding: '8px 16px', fontSize: '0.9em' }}
        >
          {isLoading ? 'Personalizing...' : isPersonalized ? 'Re-Personalize' : 'Personalize for Me'}
        </button>
      </div>

      {isPersonalized && (
        <div style={{ marginTop: '10px' }}>
          <div style={{ display: 'flex', gap: '10px', marginBottom: '10px' }}>
            <button
              onClick={() => setShowOriginal(true)}
              className={clsx('button', { 'button--primary': showOriginal })}
              style={{ padding: '5px 10px', fontSize: '0.8em' }}
            >
              Show Original
            </button>
            <button
              onClick={() => setShowOriginal(false)}
              className={clsx('button', { 'button--primary': !showOriginal })}
              style={{ padding: '5px 10px', fontSize: '0.8em' }}
            >
              Show Personalized
            </button>
          </div>

          {!showOriginal && (
            <div>
              {/* Personalization indicator badge */}
              <div style={{
                display: 'inline-block',
                padding: '4px 8px',
                backgroundColor: '#4caf50',
                color: 'white',
                borderRadius: '4px',
                fontSize: '0.8em',
                marginBottom: '10px',
                fontWeight: 'bold'
              }}>
                Personalized for: {metadata?.expertiseLevel || 'your profile'}
              </div>
              <div
                className="personalized-content"
                style={{ border: '1px solid #4caf50', borderRadius: '4px', padding: '15px', backgroundColor: '#e8f5e9' }}
                dangerouslySetInnerHTML={{ __html: personalizedContent }}
              />
            </div>
          )}

          {showOriginal && (
            <div>
              <div style={{
                display: 'inline-block',
                padding: '4px 8px',
                backgroundColor: '#2196f3',
                color: 'white',
                borderRadius: '4px',
                fontSize: '0.8em',
                marginBottom: '10px',
                fontWeight: 'bold'
              }}>
                Original Content
              </div>
              <div className="original-content-indicator">
                <span style={{ fontSize: '0.9em', color: '#666', fontStyle: 'italic' }}>
                  Currently showing original content • Personalized version available
                </span>
              </div>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

// Component to show Translate to Urdu button
const TranslateToUrduButton = ({ chapterId }) => {
  const [isLoading, setIsLoading] = useState(false);
  const [isTranslated, setIsTranslated] = useState(false);
  const [translatedContent, setTranslatedContent] = useState(null);
  const [showOriginal, setShowOriginal] = useState(true);
  const { isAuthenticated } = useAuth();

  // State for language preference with localStorage support
  const [languagePref, setLanguagePref] = useState(() => {
    if (typeof window !== 'undefined') {
      return localStorage.getItem('language_preference') || 'en';
    }
    return 'en';
  });

  const handleTranslate = async () => {
    setIsLoading(true);
    try {
      // Call the translation API
      const translateResponse = await fetch(`/api/translate/chapter/${chapterId}`, {
        method: 'POST',
        credentials: 'include',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          target_language: 'ur',
          preserve_formatting: true
        }),
      });

      if (!translateResponse.ok) {
        throw new Error('Failed to translate content');
      }

      const data = await translateResponse.json();
      setTranslatedContent(data.translatedContent);
      setIsTranslated(true);
      setShowOriginal(false);

      // Update language preference and store in localStorage
      setLanguagePref('ur');
      localStorage.setItem('language_preference', 'ur');

      // Update navigation for language
      updateNavigationForLanguage('ur');
    } catch (error) {
      console.error('Error translating content:', error);
      alert('Failed to translate content. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  // Function to toggle between original and translated content
  const toggleContent = (isOriginal) => {
    const newPref = isOriginal ? 'en' : 'ur';
    setShowOriginal(isOriginal);

    // Update language preference and store in localStorage
    setLanguagePref(newPref);
    localStorage.setItem('language_preference', newPref);

    // Update navigation for language
    updateNavigationForLanguage(newPref);
  };

  // Function to get cached Urdu translation
  const getCachedTranslation = async () => {
    try {
      const response = await fetch(`/api/translate/chapter/${chapterId}/ur`, {
        method: 'GET',
        credentials: 'include',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (response.ok) {
        const data = await response.json();
        if (data.success && data.translatedContent) {
          setTranslatedContent(data.translatedContent);
          setIsTranslated(true);
          setShowOriginal(false);

          // Update language preference and store in localStorage
          setLanguagePref('ur');
          localStorage.setItem('language_preference', 'ur');

          // Update navigation for language
          updateNavigationForLanguage('ur');
        }
      }
    } catch (error) {
      console.error('Error getting cached translation:', error);
    }
  };

  // Check for cached translation on component mount
  useEffect(() => {
    if (languagePref === 'ur' && !isTranslated) {
      getCachedTranslation();
    }
  }, []);

  return (
    <div className="translate-section" style={{ margin: '20px 0', padding: '15px', border: '1px solid #e0e0e0', borderRadius: '8px', backgroundColor: '#f9f9f9' }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '10px' }}>
        <h3 style={{ margin: 0, fontSize: '1.2em' }}>ترجمہ کریں</h3>
        <button
          onClick={handleTranslate}
          disabled={isLoading || isTranslated}
          className={clsx('button button--primary', { 'button--loading': isLoading })}
          style={{ padding: '8px 16px', fontSize: '0.9em' }}
        >
          {isLoading ? 'ترجمہ ہو رہا ہے...' : isTranslated ? 'دوبارہ ترجمہ کریں' : 'اردو میں پڑھیں'}
        </button>
      </div>

      {isTranslated && (
        <div style={{ marginTop: '10px' }}>
          <div style={{ display: 'flex', gap: '10px', marginBottom: '10px' }}>
            <button
              onClick={() => toggleContent(true)}
              className={clsx('button', { 'button--primary': showOriginal })}
              style={{ padding: '5px 10px', fontSize: '0.8em' }}
            >
              اصل مواد
            </button>
            <button
              onClick={() => toggleContent(false)}
              className={clsx('button', { 'button--primary': !showOriginal })}
              style={{ padding: '5px 10px', fontSize: '0.8em' }}
            >
              ترجمہ شدہ مواد
            </button>
          </div>

          {!showOriginal && (
            <div>
              {/* Translation indicator badge */}
              <div style={{
                display: 'inline-block',
                padding: '4px 8px',
                backgroundColor: '#ff9800',
                color: 'white',
                borderRadius: '4px',
                fontSize: '0.8em',
                marginBottom: '10px',
                fontWeight: 'bold',
                direction: 'rtl' // Right-to-left for Urdu indicator
              }}>
                اردو میں ترجمہ شدہ
              </div>
              <div
                className="translated-content"
                style={{
                  border: '1px solid #ff9800',
                  borderRadius: '4px',
                  padding: '15px',
                  backgroundColor: '#fff3e0',
                  direction: 'rtl', // Right-to-left for Urdu text
                  textAlign: 'right' // Right alignment for Urdu text
                }}
                dangerouslySetInnerHTML={{ __html: translatedContent }}
              />
            </div>
          )}

          {showOriginal && (
            <div>
              <div style={{
                display: 'inline-block',
                padding: '4px 8px',
                backgroundColor: '#2196f3',
                color: 'white',
                borderRadius: '4px',
                fontSize: '0.8em',
                marginBottom: '10px',
                fontWeight: 'bold'
              }}>
                Original Content
              </div>
              <div className="original-content-indicator">
                <span style={{ fontSize: '0.9em', color: '#666', fontStyle: 'italic' }}>
                  Currently showing original content • Translated version available
                </span>
              </div>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

// Main Content Wrapper
function ContentWrapper({ children }) {
  const { metadata, frontMatter } = useDoc();
  const location = useLocation();

  // Extract chapter ID from the slug or URL
  const chapterId = metadata?.id || location.pathname.split('/').pop() || 'unknown';

  // Extract chapter metadata
  const chapterMetadata = {
    title: metadata?.title || frontMatter?.title,
    expertiseLevel: frontMatter?.expertise_level || frontMatter?.expertiseLevel,
    prerequisites: frontMatter?.prerequisites,
    learningObjectives: frontMatter?.learning_objectives || frontMatter?.learningObjectives,
    module: frontMatter?.module,
    week: frontMatter?.week,
    estimatedTime: frontMatter?.estimated_time || frontMatter?.estimatedTime,
  };

  return (
    <div className={clsx(ThemeClassNames.docs.docWrapper, 'container')}>
      <div className="row">
        <main className={clsx(ThemeClassNames.docs.docMain, 'col', 'col--8')}>
          {/* Translation Button Section - Added at the beginning of the chapter */}
          <TranslateToUrduButton chapterId={chapterId} />

          {/* Personalize Button Section - Added at the beginning of the chapter */}
          <PersonalizeButton chapterId={chapterId} metadata={chapterMetadata} />

          {/* Original Content */}
          {children}
        </main>
      </div>
    </div>
  );
}

export default function DocItemContent({ children }) {
  return <ContentWrapper>{children}</ContentWrapper>;
}