import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import styles from './ChatWidget.module.css';

// Simple interface for message
interface Message {
  role: 'user' | 'assistant' | 'system';
  content: string;
}

export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [query, setQuery] = useState('');
  const [messages, setMessages] = useState<Message[]>([
    { role: 'assistant', content: 'Hi! I am your AI Teaching Assistant. Ask me anything about the textbook or ROS 2!' }
  ]);
  const [loading, setLoading] = useState(false);
  const [selectedTextContext, setSelectedTextContext] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const handleAskAI = (e: any) => {
      setIsOpen(true);
      const selection = e.detail.selection;
      setSelectedTextContext(selection);
      // Instead of just a system message, let's pre-fill the input
      setQuery(`Explain this: "${selection}"`);
    };
    window.addEventListener('ask-ai', handleAskAI);
    return () => window.removeEventListener('ask-ai', handleAskAI);
  }, []);

  const toggleOpen = () => setIsOpen(!isOpen);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!query.trim()) return;

    const userMsg: Message = { role: 'user', content: query };
    setMessages(prev => [...prev, userMsg]);
    setQuery('');
    setLoading(true);

    try {
      // Call FastAPI backend (proxied or direct)
      const response = await fetch('http://localhost:8000/api/v1/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ 
          query: userMsg.content, 
          history: messages.filter(m => m.role !== 'system'),
          selected_text: selectedTextContext
        })
      });

      if (!response.ok) throw new Error('Network response was not ok');
      
      const data = await response.json();
      const botMsg: Message = { role: 'assistant', content: data.answer };
      setMessages(prev => [...prev, botMsg]);
    } catch (error) {
      console.error(error);
      setMessages(prev => [...prev, { role: 'assistant', content: 'Sorry, I am having trouble connecting to the brain. Is the backend running?' }]);
    } finally {
      setLoading(false);
      // Clear selection context after one use if desired
      setSelectedTextContext(null);
    }
  };

  return (
    <div className={clsx(styles.widgetContainer)}>
      {/* Chat Button */}
      {!isOpen && (
        <button className={styles.chatButton} onClick={toggleOpen}>
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        </button>
      )}

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3>AI Tutor</h3>
            <button onClick={toggleOpen} className={styles.closeButton}>×</button>
          </div>
          
          <div className={styles.messagesArea}>
            {messages.map((msg, idx) => (
              <div key={idx} className={clsx(styles.message, msg.role === 'user' ? styles.userMsg : styles.botMsg)}>
                {msg.content}
              </div>
            ))}
            {loading && <div className={styles.loading}>Thinking...</div>}
            <div ref={messagesEndRef} />
          </div>

          <form onSubmit={handleSubmit} className={styles.inputArea}>
            <input
              type="text"
              value={query}
              onChange={(e) => setQuery(e.target.value)}
              placeholder="Ask a question..."
              className={styles.input}
            />
            <button type="submit" className={styles.sendButton} disabled={loading}>
              Send
            </button>
          </form>
        </div>
      )}
    </div>
  );
}
