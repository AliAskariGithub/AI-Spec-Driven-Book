import React from 'react';
import clsx from 'clsx';
import Card from '../Card/Card';
import styles from './Testimonials.module.css';

/**
 * Testimonials component to display success metrics and user feedback
 * Implements accessibility features and responsive design
 */
const Testimonials = ({
  title = 'What Learners Are Saying',
  subtitle = 'Real feedback from students who have used our educational resources',
  testimonials = [
    {
      id: 'testimonial1',
      quote: 'This comprehensive guide helped me understand ROS 2 concepts that took me months to grasp from other resources.',
      author: 'Sarah Johnson',
      role: 'Computer Science Student',
      company: 'Stanford University',
      avatar: '👩‍💻',
      rating: 5
    },
    {
      id: 'testimonial2',
      quote: 'The practical examples and step-by-step modules made complex robotics concepts accessible and easy to follow.',
      author: 'Michael Chen',
      role: 'AI Engineering Intern',
      company: 'NVIDIA',
      avatar: '👨‍💻',
      rating: 5
    },
    {
      id: 'testimonial3',
      quote: 'As a self-taught roboticist, these modules filled in the gaps in my knowledge and gave me confidence in my skills.',
      author: 'Alex Rivera',
      role: 'Robotics Developer',
      company: 'Boston Dynamics',
      avatar: '🧑‍🔧',
      rating: 5
    }
  ],
  className,
  ...props
}) => {
  const renderStars = (rating) => {
    return Array.from({ length: 5 }, (_, i) => (
      <span key={i} className={i < rating ? styles.starFilled : styles.starEmpty}>
        {i < rating ? '★' : '☆'}
      </span>
    ));
  };

  return (
    <section
      className={clsx(styles.testimonials, className)}
      aria-labelledby="testimonials-title"
      {...props}
    >
      <div className="container">
        <div className={styles.header}>
          <h2 id="testimonials-title" className={styles.title}>
            {title}
          </h2>
          {subtitle && (
            <p className={styles.subtitle} id="testimonials-description">
              {subtitle}
            </p>
          )}
        </div>

        <div className={styles.testimonialsGrid} role="list">
          {testimonials.map((testimonial) => (
            <div key={testimonial.id} className={styles.testimonialItem} role="listitem">
              <Card
                variant="testimonial"
                className={styles.testimonialCard}
              >
                <div className={styles.quoteSection}>
                  <div className={styles.quoteIcon}>❝</div>
                  <p className={styles.quote}>{testimonial.quote}</p>
                </div>

                <div className={styles.authorSection}>
                  <div className={styles.avatar}>{testimonial.avatar}</div>
                  <div className={styles.authorInfo}>
                    <div className={styles.authorName}>{testimonial.author}</div>
                    <div className={styles.authorRole}>{testimonial.role}</div>
                    <div className={styles.authorCompany}>{testimonial.company}</div>
                    <div className={styles.rating}>
                      {renderStars(testimonial.rating)}
                    </div>
                  </div>
                </div>
              </Card>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
};

export default Testimonials;