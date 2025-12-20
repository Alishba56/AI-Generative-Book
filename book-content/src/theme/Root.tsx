import React from 'react';
import Root from '@theme-original/Root'; // Import the original Root component
import ChatWidget from '../components/ChatWidget'; // Import our custom ChatWidget

function CustomRoot(props) {
  return (
    <>
      <Root {...props} />
      <ChatWidget /> {/* Our floating chat widget */}
    </>
  );
}

export default CustomRoot;
