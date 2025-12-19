import React from 'react';
import clsx from 'clsx';
import styles from './Card.module.css';

/**
 * Reusable Card component for feature displays
 * Implements accessibility features and responsive design
 */
const Card = ({
  children,
  title,
  description,
  icon,
  href,
  target,
  className,
  variant = 'default', // 'default', 'feature', 'testimonial'
  ...props
}) => {
  const cardClasses = clsx(
    styles.card,
    styles[variant],
    className
  );

  const cardContent = (
    <>
      {icon && (
        <div className={styles.iconContainer}>
          {typeof icon === 'string' ? (
            <span className={styles.icon}>{icon}</span>
          ) : (
            <div className={styles.icon}>{icon}</div>
          )}
        </div>
      )}
      <div className={styles.cardBody}>
        {title && <h3 className={styles.cardTitle}>{title}</h3>}
        {description && <p className={styles.cardDescription}>{description}</p>}
        {children}
      </div>
    </>
  );

  if (href) {
    return (
      <a
        href={href}
        target={target}
        rel={target === '_blank' ? 'noopener noreferrer' : undefined}
        className={cardClasses}
        {...props}
      >
        {cardContent}
      </a>
    );
  }

  return (
    <div className={cardClasses} {...props}>
      {cardContent}
    </div>
  );
};

export default Card;