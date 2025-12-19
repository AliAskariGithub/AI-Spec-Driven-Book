import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

const LearningOutcomes = ({ outcomes = [], className = '', ...props }) => {
  const [completedItems, setCompletedItems] = useState(new Set());

  const toggleComplete = (index) => {
    setCompletedItems(prev => {
      const newSet = new Set(prev);
      if (newSet.has(index)) {
        newSet.delete(index);
      } else {
        newSet.add(index);
      }
      return newSet;
    });
  };

  return (
    <section className={clsx(styles.learningOutcomes, className)} {...props}>
      <ul className={styles.learningOutcomesList}>
        {outcomes.map((outcome, index) => (
          <li
            key={index}
            className={clsx(
              styles.learningOutcomesItem,
              completedItems.has(index) && styles.completed
            )}
            onClick={() => toggleComplete(index)}
            onKeyPress={(e) => {
              if (e.key === 'Enter' || e.key === ' ') {
                e.preventDefault();
                toggleComplete(index);
              }
            }}
            role="button"
            tabIndex={0}
            aria-pressed={completedItems.has(index)}
          >
            <span 
              className={styles.learningOutcomesCheckmark} 
              aria-label={completedItems.has(index) ? "Completed" : "Not completed"}
            >
              ✓
            </span>
            <span className={styles.learningOutcomesText}>
              {outcome}
            </span>
          </li>
        ))}
      </ul>
    </section>
  );
};

export default LearningOutcomes;