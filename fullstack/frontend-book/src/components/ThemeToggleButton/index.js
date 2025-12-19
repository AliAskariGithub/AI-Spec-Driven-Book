import React from 'react';
import { useTheme } from '../ThemeSystem';
import clsx from 'clsx';

const ThemeToggleButton = ({ className = '', ...props }) => {
  const { theme, toggleTheme } = useTheme();

  return (
    <button
      onClick={toggleTheme}
      className={clsx('theme-toggle-button', className)}
      aria-label={`Switch to ${theme === 'dark' ? 'light' : 'dark'} mode`}
      {...props}
    >
      {theme === 'dark' ? '☀️' : '🌙'}
    </button>
  );
};

export default ThemeToggleButton;