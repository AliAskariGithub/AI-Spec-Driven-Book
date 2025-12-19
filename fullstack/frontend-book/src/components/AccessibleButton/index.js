import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

const AccessibleButton = ({
  children,
  onClick,
  href,
  variant = 'primary',
  className = '',
  disabled = false,
  ariaLabel,
  icon, // Icon prop (e.g., 'arrow', 'book', 'check', etc.)
  ...props
}) => {
  const buttonClasses = clsx(
    styles.accessibleButton,
    `button`,
    `button--${variant}`,
    { 'button--disabled': disabled },
    className
  );

  // Map icon names to actual icons
  const getIcon = (iconName) => {
    const iconMap = {
      'arrow': '→',
      'book': '📚',
      'check': '✓',
      'star': '⭐',
      'play': '▶',
      'info': 'ℹ',
      'download': '⬇',
      'external': '↗',
    };
    return iconMap[iconName] || '';
  };

  const iconElement = icon ? (
    <span className={styles.buttonIcon} aria-hidden="true">
      {getIcon(icon)}
    </span>
  ) : null;

  // Determine if it should be a link or button element
  if (href) {
    return (
      <Link
        className={buttonClasses}
        to={href}
        aria-label={ariaLabel}
        aria-disabled={disabled}
        {...props}
      >
        {iconElement}
        {children}
      </Link>
    );
  }

  return (
    <button
      className={buttonClasses}
      onClick={onClick}
      disabled={disabled}
      aria-label={ariaLabel}
      aria-disabled={disabled}
      {...props}
    >
      {iconElement}
      {children}
    </button>
  );
};

export default AccessibleButton;