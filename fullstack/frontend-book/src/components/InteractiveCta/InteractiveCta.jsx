import React, { useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Button from '../Button/Button';
import styles from './InteractiveCta.module.css';

/**
 * Interactive CTA component for learning path selection
 * Implements accessibility features and interactive hover/focus states
 */
const InteractiveCta = ({
  title = 'Choose Your Learning Path',
  subtitle = 'Select the path that matches your current skill level',
  ctaOptions = [
    {
      id: 'beginner',
      title: 'For Beginners',
      description: 'Start with the basics and build your knowledge step by step',
      link: '/docs/intro',
      color: 'var(--custom-color-primary)',
      icon: '🎓'
    },
    {
      id: 'advanced',
      title: 'For Advanced Users',
      description: 'Dive deep into advanced topics and best practices',
      link: '/docs/advanced',
      color: 'var(--custom-color-accent)',
      icon: '🚀'
    },
    {
      id: 'examples',
      title: 'View Examples',
      description: 'Explore real-world examples and use cases',
      link: '/docs/examples',
      color: 'var(--custom-color-primary-light)',
      icon: '💡'
    }
  ],
  className,
  ...props
}) => {
  const [hoveredCard, setHoveredCard] = useState(null);
  const [focusedCard, setFocusedCard] = useState(null);

  const handleCardFocus = (cardId) => {
    setFocusedCard(cardId);
  };

  const handleCardBlur = () => {
    setFocusedCard(null);
  };

  return (
    <section
      className={clsx(styles.interactiveCta, className)}
      aria-labelledby="learning-path-title"
      {...props}
    >
      <div className="container">
        <div className={styles.header}>
          <h2 id="learning-path-title" className={styles.title}>
            {title}
          </h2>
          <p className={styles.subtitle} id="learning-path-description">
            {subtitle}
          </p>
        </div>

        <div
          className={styles.ctaGrid}
          role="group"
          aria-labelledby="learning-path-title"
        >
          {ctaOptions.map((option) => (
            <div
              key={option.id}
              className={clsx(
                styles.ctaCard,
                {
                  [styles.hovered]: hoveredCard === option.id,
                  [styles.focused]: focusedCard === option.id
                }
              )}
              style={{ borderLeft: `4px solid ${option.color}` }}
              onMouseEnter={() => setHoveredCard(option.id)}
              onMouseLeave={() => setHoveredCard(null)}
              onFocus={() => handleCardFocus(option.id)}
              onBlur={handleCardBlur}
              tabIndex={0}
              role="button"
              aria-describedby={`cta-${option.id}-desc`}
              onKeyDown={(e) => {
                if (e.key === 'Enter' || e.key === ' ') {
                  window.location.href = option.link;
                }
              }}
            >
              <div className={styles.cardIcon}>
                {option.icon}
              </div>
              <h3 id={`cta-${option.id}-title`} className={styles.cardTitle}>
                {option.title}
              </h3>
              <p id={`cta-${option.id}-desc`} className={styles.cardDescription}>
                {option.description}
              </p>
              <Button
                variant="primary"
                href={option.link}
                style={{ backgroundColor: option.color, borderColor: option.color }}
                aria-describedby={`cta-${option.id}-title cta-${option.id}-desc`}
              >
                Get Started
              </Button>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
};

export default InteractiveCta;