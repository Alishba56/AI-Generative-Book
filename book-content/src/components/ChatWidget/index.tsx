import React, { useState, useRef, useEffect } from 'react';
import ReactMarkdown from 'react-markdown';
import styles from './styles.module.css';

// The backend URL should be an environment variable in a real app
const BACKEND_URL = 'http://localhost:8000';

const ChatIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
  </svg>
);

const SendIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <line x1="22" y1="2" x2="11" y2="13"></line>
    <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
  </svg>
);


function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<{ text: string; sender: 'user' | 'bot' }[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(scrollToBottom, [messages]);

  const toggleChat = () => setIsOpen(!isOpen);

  const handleSendMessage = async () => {
    if (input.trim() === '') return;

    const userMessage = { text: input, sender: 'user' as const };
    setMessages((prevMessages) => [...prevMessages, userMessage]);
    setInput('');
    setIsLoading(true);
    
    // Add a placeholder for the bot's response
    setMessages((prevMessages) => [
      ...prevMessages,
      { text: '', sender: 'bot' as const },
    ]);

    try {
      const response = await fetch(`${BACKEND_URL}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Accept': 'text/event-stream',
        },
        body: JSON.stringify({ query: userMessage.text }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      
      if (!response.body) {
        throw new Error('Response body is null');
      }

      // Handle the stream
      const reader = response.body.getReader();
      const decoder = new TextDecoder();
      
      let done = false;
      while (!done) {
        const { value, done: readerDone } = await reader.read();
        done = readerDone;
        const chunk = decoder.decode(value, { stream: true });
        
        // Update the last message (the bot's response) with the new chunk
        setMessages((prevMessages) => {
          const lastMessage = prevMessages[prevMessages.length - 1];
          if (lastMessage && lastMessage.sender === 'bot') {
            const updatedMessages = [...prevMessages];
            updatedMessages[prevMessages.length - 1] = {
              ...lastMessage,
              text: lastMessage.text + chunk,
            };
            return updatedMessages;
          }
          return prevMessages;
        });
      }

    } catch (error) {
      console.error('Error sending message:', error);
      setMessages((prevMessages) => {
        // Find the bot placeholder and update it with an error message
        const lastMessageIndex = prevMessages.length - 1;
        if (lastMessageIndex >= 0 && prevMessages[lastMessageIndex].sender === 'bot' && prevMessages[lastMessageIndex].text === '') {
          const updatedMessages = [...prevMessages];
          updatedMessages[lastMessageIndex] = {
            ...updatedMessages[lastMessageIndex],
            text: 'Sorry, something went wrong. Please try again.',
          };
          return updatedMessages;
        }
        // If no placeholder, add a new error message
        return [...prevMessages, { text: 'Sorry, something went wrong. Please try again.', sender: 'bot' }];
      });
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.chatWidgetContainer}>
      <button className={styles.chatToggleButton} onClick={toggleChat}>
        <ChatIcon />
      </button>

      {isOpen && (
        <div className={`${styles.chatWindow} ${isOpen ? styles.open : ''}`}>
          <div className={styles.chatHeader}>
            <h3>AI Robotics Chat</h3>
            <button className={styles.chatCloseButton} onClick={toggleChat}>&times;</button>
          </div>
          <div className={styles.chatMessages}>
             {messages.map((msg, index) => (
              <div key={index} className={`${styles.message} ${styles[msg.sender]}`}>
                <ReactMarkdown>{msg.text}</ReactMarkdown>
              </div>
            ))}
            {isLoading && messages.length > 0 && messages[messages.length -1].sender === 'bot' && (
              <div className={`${styles.message} ${styles.bot}`}>
                <div className={styles.loadingDots}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          <div className={styles.chatInputContainer}>
            <input
              type="text"
              className={styles.chatInput}
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={(e) => e.key === 'Enter' && !isLoading && handleSendMessage()}
              placeholder="Ask a question..."
              disabled={isLoading}
            />
            <button className={styles.sendButton} onClick={handleSendMessage} disabled={isLoading}>
              <SendIcon />
            </button>
          </div>
        </div>
      )}
    </div>
  );
}

export default ChatWidget;