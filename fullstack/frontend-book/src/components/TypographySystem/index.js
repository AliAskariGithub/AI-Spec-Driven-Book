import React from 'react';
import clsx from 'clsx';

const TypographyWrapper = ({
  variant = 'body',
  pageType = 'landing',
  children,
  className = '',
  element: Element = null,
  ...props
}) => {
  // Determine font family based on page type and variant
  const getFontFamily = () => {
    if (variant === 'heading') {
      // Always use Archivo for headings regardless of page type
      return 'Archivo, system-ui, -apple-system, sans-serif';
    } else {
      // Use different body fonts based on page type
      if (pageType === 'landing') {
        return 'General Sans, system-ui, -apple-system, sans-serif';
      } else {
        // textbook page type
        return '"Source Serif 4", serif';
      }
    }
  };

  // Determine default element based on variant
  const defaultElement = variant === 'heading' ? 'h2' : 'p';
  const elementType = Element || defaultElement;

  // Apply font family through inline style
  const style = {
    fontFamily: getFontFamily(),
    ...props.style,
  };

  // Apply classes for proper styling
  const classes = clsx(
    className,
    `typography-${variant}`,
    `typography-${pageType}`
  );

  return React.createElement(
    elementType,
    {
      className: classes,
      style,
      ...props,
    },
    children
  );
};

export default TypographyWrapper;