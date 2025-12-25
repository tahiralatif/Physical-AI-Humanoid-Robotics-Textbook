import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './TranslateToggle.module.css';

interface TranslateToggleProps {
  chapterId: string;
  onTranslate: (translated: string | null) => void;
  getContent: () => string;
}

export default function TranslateToggle({ chapterId, onTranslate, getContent }: TranslateToggleProps) {
  const [isUrdu, setIsUrdu] = useState(false);
  const [loading, setLoading] = useState(false);

  const handleToggle = async () => {
    if (isUrdu) {
      setIsUrdu(false);
      onTranslate(null); // Reset to English
      return;
    }

    const content = getContent();
    if (!content) {
      alert("Could not find content to translate.");
      return;
    }

    setLoading(true);
    try {
      const response = await fetch('http://localhost:8000/api/v1/translate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ 
          content: content,
          target_lang: 'Urdu' 
        })
      });

      if (!response.ok) throw new Error('Translation failed');
      
      const data = await response.json();
      setIsUrdu(true);
      onTranslate(data.translated_content);
    } catch (error) {
      console.error(error);
      alert('Translation failed. Is the backend running?');
    } finally {
      setLoading(false);
    }
  };

  return (
    <button 
      onClick={handleToggle} 
      className={clsx(styles.button, isUrdu && styles.active)}
      disabled={loading}
    >
      {loading ? 'Translating...' : (isUrdu ? 'Show English' : 'اردو میں ترجمہ کریں')}
    </button>
  );
}
