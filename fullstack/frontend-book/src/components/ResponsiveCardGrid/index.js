import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

const ResponsiveCardGrid = ({
  children,
  className = '',
  columns = { mobile: 1, tablet: 2, desktop: 3 },
  gap = '2rem',
  ...props
}) => {
  return (
    <div
      className={clsx(styles.responsiveCardGrid, className)}
      style={{
        gap: gap,
        ...props.style
      }}
      {...props}
    >
      {children}
    </div>
  );
};

export default ResponsiveCardGrid;