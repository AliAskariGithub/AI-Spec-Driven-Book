# Quickstart Guide: Landing Page Improvement for Docusaurus Site

## Overview

This quickstart guide provides step-by-step instructions to implement the landing page improvement for the Docusaurus site. Follow these steps to create an engaging, interactive landing page that clearly explains the site's purpose to developers and learners.

## Prerequisites

- Node.js v18+ installed
- npm or yarn package manager
- Git for version control
- Code editor (VS Code recommended)
- Basic knowledge of React and JSX

## Step 1: Set Up Development Environment

### 1.1 Clone the Repository
```bash
git clone <repository-url>
cd ai-book/fullstack/frontend-book
```

### 1.2 Install Dependencies
```bash
npm install
# or
yarn install
```

### 1.3 Start Development Server
```bash
npm run start
# or
yarn start
```

The development server will start at `http://localhost:3000`.

## Step 2: Create Landing Page Components

### 2.1 Create Homepage Features Component

First, let's enhance the existing HomepageFeatures component or create a new one:

```jsx
// src/components/LandingPageFeatures/index.js
import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Easy to Learn',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Designed for developers and learners to quickly understand
        and implement documentation best practices.
      </>
    ),
  },
  {
    title: 'Focus on What Matters',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Let us handle the documentation infrastructure so you can
        focus on your content and learning objectives.
      </>
    ),
  },
  {
    title: 'Powered by React',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Build modern documentation sites using React components
        and industry-standard tools.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
```

### 2.2 Create Styles for Homepage Features

```css
/* src/components/LandingPageFeatures/styles.module.css */
.features {
  display: flex;
  align-items: center;
  padding: 2rem 0;
  width: 100%;
}

.featureSvg {
  height: 200px;
  width: 200px;
}
```

## Step 3: Update the Landing Page

### 3.1 Modify the Main Landing Page Component

Replace the content in `src/pages/index.js` with an enhanced landing page:

```jsx
// src/pages/index.js
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/LandingPageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Get Started - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={siteConfig.title}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
```

### 3.2 Update Landing Page Styles

Enhance the styles in `src/pages/index.module.css`:

```css
/* src/pages/index.module.css */
/**
 * CSS files with the .module.css suffix will be treated as CSS modules
 * and scoped locally.
 */

.heroBanner {
  padding: 4rem 0;
  text-align: center;
  position: relative;
  overflow: hidden;
}

@media screen and (max-width: 996px) {
  .heroBanner {
    padding: 2rem;
  }
}

.buttons {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 1rem;
  flex-wrap: wrap;
}

/* Enhanced styles for interactive elements */
.buttons a {
  transition: all 0.3s ease;
  transform: translateY(0);
}

.buttons a:hover {
  transform: translateY(-3px);
  box-shadow: 0 10px 20px rgba(0,0,0,0.1);
}
```

## Step 4: Add Interactive Elements

### 4.1 Create an Interactive CTA Component

```jsx
// src/components/InteractiveCta/index.js
import React, { useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

export default function InteractiveCta() {
  const [hoveredButton, setHoveredButton] = useState(null);

  const ctaOptions = [
    {
      id: 'beginner',
      title: 'For Beginners',
      description: 'Start with the basics and build your knowledge step by step',
      link: '/docs/intro',
      color: '#42c542'
    },
    {
      id: 'advanced',
      title: 'For Advanced Users',
      description: 'Dive deep into advanced topics and best practices',
      link: '/docs/advanced',
      color: '#6e5494'
    },
    {
      id: 'examples',
      title: 'View Examples',
      description: 'Explore real-world examples and use cases',
      link: '/docs/examples',
      color: '#29adff'
    }
  ];

  return (
    <section className={styles.interactiveCta}>
      <div className="container">
        <h2 className={styles.sectionTitle}>Choose Your Learning Path</h2>
        <p className={styles.sectionSubtitle}>Select the path that matches your current skill level</p>

        <div className={styles.ctaGrid}>
          {ctaOptions.map((option) => (
            <div
              key={option.id}
              className={clsx(styles.ctaCard, {
                [styles.hovered]: hoveredButton === option.id
              })}
              onMouseEnter={() => setHoveredButton(option.id)}
              onMouseLeave={() => setHoveredButton(null)}
              style={{
                borderLeft: `4px solid ${option.color}`
              }}
            >
              <h3>{option.title}</h3>
              <p>{option.description}</p>
              <Link
                className="button button--primary"
                to={option.link}
                style={{ backgroundColor: option.color, borderColor: option.color }}
              >
                Get Started
              </Link>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}
```

### 4.2 Create Styles for Interactive CTA

```css
/* src/components/InteractiveCta/styles.module.css */
.interactiveCta {
  padding: 4rem 0;
  background-color: #f9f9f9;
}

.sectionTitle {
  text-align: center;
  margin-bottom: 1rem;
  font-size: 2.5rem;
}

.sectionSubtitle {
  text-align: center;
  margin-bottom: 3rem;
  font-size: 1.2rem;
  color: #666;
}

.ctaGrid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 2rem;
  max-width: 1200px;
  margin: 0 auto;
}

.ctaCard {
  background: white;
  padding: 2rem;
  border-radius: 8px;
  box-shadow: 0 2px 10px rgba(0,0,0,0.1);
  transition: all 0.3s ease;
  border-left: 4px solid #42c542;
}

.ctaCard.hovered {
  transform: translateY(-10px);
  box-shadow: 0 12px 20px rgba(0,0,0,0.15);
}

.ctaCard h3 {
  margin-top: 0;
  color: #333;
}

.ctaCard p {
  color: #666;
  margin-bottom: 1.5rem;
}

.ctaCard .button {
  text-decoration: none;
  padding: 0.75rem 1.5rem;
  display: inline-block;
  transition: all 0.3s ease;
}

.ctaCard .button:hover {
  transform: scale(1.05);
}
```

## Step 5: Integrate Interactive Elements

Update the main landing page to include the interactive CTA:

```jsx
// Updated src/pages/index.js
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/LandingPageFeatures';
import InteractiveCta from '@site/src/components/InteractiveCta';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Get Started - 5min ⏱️
          </Link>
          <Link
            className="button button--primary button--lg"
            to="/docs/examples">
            View Examples
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <InteractiveCta />
      </main>
    </Layout>
  );
}
```

## Step 6: Add Accessibility Features

### 6.1 Update Components with Accessibility Attributes

Enhance the InteractiveCta component with accessibility features:

```jsx
// Enhanced accessibility version of InteractiveCta/index.js
import React, { useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

export default function InteractiveCta() {
  const [focusedCard, setFocusedCard] = useState(null);

  const ctaOptions = [
    {
      id: 'beginner',
      title: 'For Beginners',
      description: 'Start with the basics and build your knowledge step by step',
      link: '/docs/intro',
      color: '#42c542'
    },
    {
      id: 'advanced',
      title: 'For Advanced Users',
      description: 'Dive deep into advanced topics and best practices',
      link: '/docs/advanced',
      color: '#6e5494'
    },
    {
      id: 'examples',
      title: 'View Examples',
      description: 'Explore real-world examples and use cases',
      link: '/docs/examples',
      color: '#29adff'
    }
  ];

  return (
    <section
      className={styles.interactiveCta}
      aria-labelledby="learning-path-title"
    >
      <div className="container">
        <h2
          id="learning-path-title"
          className={styles.sectionTitle}
        >
          Choose Your Learning Path
        </h2>
        <p
          className={styles.sectionSubtitle}
          id="learning-path-description"
        >
          Select the path that matches your current skill level
        </p>

        <div
          className={styles.ctaGrid}
          role="group"
          aria-labelledby="learning-path-title"
        >
          {ctaOptions.map((option) => (
            <div
              key={option.id}
              className={clsx(styles.ctaCard, {
                [styles.focused]: focusedCard === option.id
              })}
              style={{
                borderLeft: `4px solid ${option.color}`
              }}
              onFocus={() => setFocusedCard(option.id)}
              onBlur={() => setFocusedCard(null)}
              tabIndex={0}
              role="button"
              aria-describedby={`cta-${option.id}-desc`}
            >
              <h3 id={`cta-${option.id}-title`}>{option.title}</h3>
              <p id={`cta-${option.id}-desc`}>{option.description}</p>
              <Link
                className="button button--primary"
                to={option.link}
                style={{ backgroundColor: option.color, borderColor: option.color }}
                aria-describedby={`cta-${option.id}-title cta-${option.id}-desc`}
              >
                Get Started
              </Link>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}
```

## Step 7: Add Responsive Design

### 7.1 Update CSS for Mobile Responsiveness

Enhance the CSS to ensure proper mobile display:

```css
/* Enhanced responsive styles in src/components/InteractiveCta/styles.module.css */
@media screen and (max-width: 768px) {
  .interactiveCta {
    padding: 2rem 0;
  }

  .sectionTitle {
    font-size: 2rem;
  }

  .sectionSubtitle {
    font-size: 1rem;
    padding: 0 1rem;
  }

  .ctaGrid {
    grid-template-columns: 1fr;
    gap: 1.5rem;
    padding: 0 1rem;
  }

  .ctaCard {
    padding: 1.5rem;
  }

  .ctaCard.hovered {
    transform: none; /* Disable hover effects on mobile */
  }
}

@media screen and (max-width: 480px) {
  .sectionTitle {
    font-size: 1.75rem;
  }

  .ctaCard h3 {
    font-size: 1.25rem;
  }
}
```

## Step 8: Testing the Implementation

### 8.1 Manual Testing Checklist

- [ ] Landing page loads without errors
- [ ] Hero section displays correctly with title and subtitle
- [ ] Call-to-action buttons are visible and clickable
- [ ] Feature cards display properly with descriptions
- [ ] Interactive elements respond to hover and focus
- [ ] Responsive design works on different screen sizes
- [ ] Accessibility features work with screen readers
- [ ] All links navigate to correct destinations

### 8.2 Run Automated Tests

```bash
npm run test
# or
yarn test
```

## Step 9: Build and Deploy

### 9.1 Build for Production

```bash
npm run build
# or
yarn build
```

### 9.2 Serve Locally for Testing

```bash
npm run serve
# or
yarn serve
```

## Troubleshooting

### Common Issues and Solutions

1. **Component not rendering**
   - Check file paths and import statements
   - Verify component exports are correct
   - Ensure Docusaurus server is restarted after file changes

2. **Styles not applying**
   - Verify CSS module file naming (filename.module.css)
   - Check class names are properly referenced
   - Clear browser cache if needed

3. **Interactive elements not working**
   - Verify React state hooks are properly implemented
   - Check for JavaScript errors in browser console
   - Ensure React is properly imported in components

## Next Steps

1. Review the landing page with stakeholders
2. Conduct user testing to validate the value proposition
3. Optimize images and assets for performance
4. Implement analytics to track user engagement
5. Plan A/B tests for different CTA strategies