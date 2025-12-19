import React from 'react';
import clsx from 'clsx';
import Card from '../Card/Card';
import styles from './FeatureSection.module.css';

/**
 * FeatureSection component to display key documentation benefits
 * Implements responsive grid layout and accessibility features
 */
const FeatureSection = ({
  title = 'What You\'ll Learn',
  subtitle = 'Explore the comprehensive modules designed for AI and CS students',
  features = [
    {
      id: 'module1',
      title: 'Module 1: The Robotic Nervous System',
      description: 'Learn about ROS 2, the middleware that connects robotic systems and enables communication between components.',
      icon: '🤖',
      href: '/docs/module-1'
    },
    {
      id: 'module2',
      title: 'Module 2: The Digital Twin',
      description: 'Master Gazebo and Unity simulation environments for testing and validating robotic systems in virtual worlds.',
      icon: '🎮',
      href: '/docs/module-2'
    },
    {
      id: 'module3',
      title: 'Module 3: Isaac Sim',
      description: 'Explore NVIDIA Isaac Sim for advanced robotics simulation and AI development in photorealistic environments.',
      icon: '🚀',
      href: '/docs/module-3'
    }
  ],
  layout = 'grid', // 'grid', 'list', or 'carousel'
  className,
  ...props
}) => {
  const sectionClasses = clsx(
    styles.featureSection,
    styles[layout],
    className
  );

  return (
    <section
      className={sectionClasses}
      aria-labelledby="feature-section-title"
      {...props}
    >
      <div className="container">
        <div className={styles.header}>
          <h2 id="feature-section-title" className={styles.title}>
            {title}
          </h2>
          {subtitle && (
            <p className={styles.subtitle}>
              {subtitle}
            </p>
          )}
        </div>

        <div className={styles.featuresGrid} role="list">
          {features.map((feature) => (
            <div key={feature.id} className={styles.featureItem} role="listitem">
              <Card
                variant="feature"
                title={feature.title}
                description={feature.description}
                icon={feature.icon}
                href={feature.href}
                className={styles.featureCard}
              />
            </div>
          ))}
        </div>
      </div>
    </section>
  );
};

export default FeatureSection;