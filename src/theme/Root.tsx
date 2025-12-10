import React from 'react';
import { ChatWidget } from '../components/ChatWidget/ChatWidget';

// Global Root component that wraps the entire app
const Root = ({ children }: { children: React.ReactNode }) => {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
};

export default Root;