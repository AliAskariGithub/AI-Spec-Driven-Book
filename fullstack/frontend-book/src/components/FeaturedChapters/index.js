import React, { useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

const FeaturedChapters = ({ chapters = [], className = '', ...props }) => {
  const [hoveredCard, setHoveredCard] = useState(null);

  return (
    <section className={clsx(styles.featuredChapters, className)} {...props}>
      <div className={styles.chaptersGrid}>
        {chapters.map((chapter, index) => (
          <div
            key={chapter.id}
            className={clsx(
              styles.chapterCard,
              hoveredCard === chapter.id && styles.hovered
            )}
            onMouseEnter={() => setHoveredCard(chapter.id)}
            onMouseLeave={() => setHoveredCard(null)}
            style={{ animationDelay: `${index * 0.1}s` }}
          >
            <div className={styles.chapterNumber}>
              {String(index + 1).padStart(2, '0')}
            </div>
            <div className={styles.chapterContent}>
              <h3 className={styles.chapterTitle}>{chapter.title}</h3>
              <p className={styles.chapterDescription}>{chapter.description}</p>
              <Link
                className={styles.chapterLink}
                to={chapter.path}
                aria-label={`Read more about ${chapter.title}`}
              >
                Read Chapter
                <span className={styles.arrow} aria-hidden="true">→</span>
              </Link>
            </div>
            <div className={styles.cardGlow} aria-hidden="true" />
          </div>
        ))}
      </div>
    </section>
  );
};

export default FeaturedChapters;