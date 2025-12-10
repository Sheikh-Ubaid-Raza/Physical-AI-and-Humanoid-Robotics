import React, { useEffect } from 'react';
import { ChatWidget } from '../components/ChatWidget/ChatWidget';

// Default theme Root component that wraps the entire app
const Root = ({ children }: { children: React.ReactNode }) => {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
};

export default Root;