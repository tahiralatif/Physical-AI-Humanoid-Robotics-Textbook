import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './PersonalizeButton.module.css';

interface PersonalizeButtonProps {
  chapterId: string;
  onPersonalize: (content: string | null) => void;
  getContent: () => string;
}

export default function PersonalizeButton({ chapterId, onPersonalize, getContent }: PersonalizeButtonProps) {
  const [loading, setLoading] = useState(false);
  const [personalized, setPersonalized] = useState(false);

  const handlePersonalize = async () => {
    if (personalized) {
      setPersonalized(false);
      onPersonalize(null);
      return;
    }

    const content = getContent();
    if (!content) {
      alert("Could not find content to personalize.");
      return;
    }

    setLoading(true);
    try {
      // In a real app, you'd get this from context/auth
      const userContext = "Beginner with no hardware"; 
      
      const response = await fetch('http://localhost:8000/api/v1/personalize', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ 
          chapter_id: chapterId,
          content: content,
          user_context: userContext 
        })
      });

      if (!response.ok) throw new Error('Personalization failed');
      
      const data = await response.json();
      setPersonalized(true);
      onPersonalize(data.personalized_content);
    } catch (error) {
      console.error(error);
      alert('Personalization failed. Is the backend running?');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.container}>
      <button 
        onClick={handlePersonalize} 
        disabled={loading}
        className={clsx(styles.button, loading && styles.loading, personalized && styles.active)}
      >
        {loading ? 'Personalizing...' : (personalized ? 'Show Original' : '✨ Personalize for Me')}
      </button>
    </div>
  );
}
