import React from 'react';
import clsx from 'clsx';
import styles from './IsaacHighlight.module.css';

const IsaacIcon = () => (
  <span className={styles.isaacIcon}>🚀</span>
);

export default function IsaacHighlight({title, children, type = 'default'}) {
  return (
    <div className={clsx(styles.isaacHighlight, styles[`isaacHighlight--${type}`])}>
      <div className={styles.isaacHeader}>
        <IsaacIcon />
        {title && <h3 className={styles.isaacTitle}>{title}</h3>}
      </div>
      <div className={styles.isaacContent}>
        {children}
      </div>
    </div>
  );
}