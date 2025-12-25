import React, {type ReactNode, useState, useEffect} from 'react';
import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type {WrapperProps} from '@docusaurus/types';
import PersonalizeButton from '@site/src/components/PersonalizeButton';
import TranslateToggle from '@site/src/components/TranslateToggle';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import ReactMarkdown from 'react-markdown';

type Props = WrapperProps<typeof ContentType>;

export default function ContentWrapper(props: Props): ReactNode {
  const { metadata, contentTitle } = useDoc();
  const [aiContent, setAiContent] = useState<string | null>(null);
  const [isRTL, setIsRTL] = useState(false);
  const [selection, setSelection] = useState<{text: string, x: number, y: number} | null>(null);

  const handleAiUpdate = (newContent: string | null, rtl: boolean = false) => {
    setAiContent(newContent);
    setIsRTL(rtl);
  };

  useEffect(() => {
    const handleMouseUp = () => {
      const selected = window.getSelection()?.toString().trim();
      if (selected && selected.length > 10) {
        const range = window.getSelection()?.getRangeAt(0);
        const rect = range?.getBoundingClientRect();
        if (rect) {
          setSelection({
            text: selected,
            x: rect.left + window.scrollX,
            y: rect.top + window.scrollY - 40
          });
        }
      } else {
        setSelection(null);
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    return () => document.removeEventListener('mouseup', handleMouseUp);
  }, []);

  const askAboutSelection = () => {
    if (!selection) return;
    // Dispatch a custom event that ChatWidget listens to
    const event = new CustomEvent('ask-ai', { detail: { selection: selection.text } });
    window.dispatchEvent(event);
    setSelection(null);
  };

  // Extract content text for AI processing
  // This is a bit tricky in Docusaurus, but we can target the main content
  const getDocContent = () => {
    const mainContent = document.querySelector('.theme-doc-markdown');
    return mainContent?.textContent || "";
  };

  return (
    <>
      <div style={{ display: 'flex', gap: '10px', marginBottom: '20px', flexWrap: 'wrap' }}>
        <PersonalizeButton 
          chapterId={metadata.id} 
          onPersonalize={(content) => handleAiUpdate(content)}
          getContent={getDocContent}
        />
        <TranslateToggle 
          chapterId={metadata.id} 
          onTranslate={(content) => handleAiUpdate(content, !!content)}
          getContent={getDocContent}
        />
      </div>

      {selection && (
        <button 
          onClick={askAboutSelection}
          style={{
            position: 'absolute',
            left: selection.x,
            top: selection.y,
            zIndex: 1000,
            backgroundColor: 'var(--ifm-color-primary)',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            padding: '4px 8px',
            fontSize: '12px',
            cursor: 'pointer',
            boxShadow: '0 2px 4px rgba(0,0,0,0.2)'
          }}
        >
          🤖 Ask AI about this
        </button>
      )}

      <div className={isRTL ? 'translated-content' : ''} dir={isRTL ? 'rtl' : 'ltr'}>
        {aiContent ? (
          <div className="prose max-w-none ai-generated">
            <ReactMarkdown>{aiContent}</ReactMarkdown>
          </div>
        ) : (
          <Content {...props} />
        )}
      </div>
    </>
  );
}
