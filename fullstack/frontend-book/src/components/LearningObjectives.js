import React from 'react';
import clsx from 'clsx';
import styles from './LearningObjectives.module.css';

const LearningObjectives = ({ children }) => {
  return (
    <div className={clsx('alert alert--info', styles.learningObjectives)}>
      <div className={styles.learningObjectivesHeader}>
        <svg className={styles.learningObjectivesIcon} viewBox="0 0 24 24" width="24" height="24">
          <path fill="currentColor" d="M12,3L1,9L12,15L21,10.09V17H23V9M5,13.18V17.18L12,21.5L19,17.18V13.18L12,17.5L5,13.18Z" />
        </svg>
        <h3 className={styles.learningObjectivesTitle}>Learning Objectives</h3>
      </div>
      <ul className={styles.learningObjectivesList}>
        {children}
      </ul>
    </div>
  );
};

export default LearningObjectives;