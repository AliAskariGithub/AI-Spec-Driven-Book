import React from 'react';
import clsx from 'clsx';
import styles from './Button.module.css';

/**
 * Reusable Button component with primary/secondary variants
 * Follows accessibility best practices with proper ARIA attributes
 */
const Button = ({
  children,
  variant = 'primary', // 'primary', 'secondary', or 'outline'
  size = 'medium',     // 'small', 'medium', or 'large'
  disabled = false,
  href,
  target,
  rel,
  onClick,
  className,
  type = 'button',
  ...props
}) => {
  const buttonClasses = clsx(
    styles.button,
    styles[variant],
    styles[size],
    { [styles.disabled]: disabled },
    className
  );

  // Determine if it's a link or button element
  const isLink = href && !disabled;

  if (isLink) {
    // Render as link element
    return (
      <a
        href={href}
        target={target}
        rel={rel || (target === '_blank' ? 'noopener noreferrer' : undefined)}
        className={buttonClasses}
        aria-disabled={disabled}
        onClick={disabled ? (e) => e.preventDefault() : undefined}
        {...props}
      >
        <span className={styles.buttonContent}>{children}</span>
      </a>
    );
  } else {
    // Render as button element
    return (
      <button
        type={type}
        className={buttonClasses}
        disabled={disabled}
        onClick={disabled ? undefined : onClick}
        {...props}
      >
        <span className={styles.buttonContent}>{children}</span>
      </button>
    );
  }
};

export default Button;