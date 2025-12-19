import React, { useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

const FeatureCard = ({ 
  title, 
  description, 
  icon: Icon, 
  link, 
  color = '#6C3BAA', 
  className = '' 
}) => {
  const [isHovered, setIsHovered] = useState(false);

  const CardContent = () => (
    <>
      <div className={styles.cardBackground} style={{ background: color }} />
      <div className={styles.cardGlow} style={{ background: `radial-gradient(circle, ${color}20, transparent 70%)` }} />
      
      <div className={styles.iconContainer} style={{ background: `${color}15` }}>
        {Icon ? (
          <Icon className={styles.icon} style={{ color }} />
        ) : (
          <div className={styles.iconPlaceholder} style={{ background: color }}>
            <span className={styles.iconText}>✦</span>
          </div>
        )}
      </div>
      
      <div className={styles.cardContent}>
        <h3 className={styles.title}>{title}</h3>
        <p className={styles.description}>{description}</p>
        
        {link && (
          <div className={styles.linkContainer}>
            <span className={styles.linkText} style={{ color }}>
              Learn More
            </span>
            <span className={styles.arrow} style={{ color }}>→</span>
          </div>
        )}
      </div>
      
      <div className={styles.cardBorder} style={{ borderColor: color }} />
    </>
  );

  const commonProps = {
    className: clsx(styles.featureCard, className, {
      [styles.featureCardHovered]: isHovered
    }),
    onMouseEnter: () => setIsHovered(true),
    onMouseLeave: () => setIsHovered(false),
    onFocus: () => setIsHovered(true),
    onBlur: () => setIsHovered(false),
  };

  if (link) {
    return (
      <Link
        {...commonProps}
        to={link}
        style={{ textDecoration: 'none', color: 'inherit' }}
        aria-label={`Learn more about ${title}`}
      >
        <CardContent />
      </Link>
    );
  }

  return (
    <div {...commonProps} tabIndex={0} role="article">
      <CardContent />
    </div>
  );
};

export default FeatureCard;