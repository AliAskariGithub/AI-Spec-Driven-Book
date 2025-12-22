# Quickstart Guide: Docusaurus UI Upgrade Implementation

## Overview
This guide provides step-by-step instructions to implement the Docusaurus UI Upgrade with modern design, improved typography, dark/light mode, and enhanced navigation as specified in the feature requirements.

## Prerequisites
- Node.js v18+ installed
- npm or yarn package manager
- Git for version control
- Code editor (VS Code recommended)
- Basic knowledge of React and Docusaurus

## Step 1: Setup Development Environment

### 1.1 Clone and Navigate to Project
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

## Step 2: Font Setup

### 2.1 Add Font Files
Ensure the following font files are in the `static/fonts/` directory:
- Archivo (for headings) - both landing and textbook pages
- General Sans (for landing page body text)
- SourceSerif4 (for textbook page body text)

### 2.2 Create Font CSS
Create `src/css/fonts.css` with the following:

```css
/* Archivo - for headings */
@font-face {
  font-family: 'Archivo';
  font-style: normal;
  font-weight: 400;
  font-display: swap;
  src: url('/fonts/Archivo/Archivo-Regular.woff2') format('woff2'),
       url('/fonts/Archivo/Archivo-Regular.woff') format('woff');
}

@font-face {
  font-family: 'Archivo';
  font-style: normal;
  font-weight: 600;
  font-display: swap;
  src: url('/fonts/Archivo/Archivo-SemiBold.woff2') format('woff2'),
       url('/fonts/Archivo/Archivo-SemiBold.woff') format('woff');
}

/* General Sans - for landing page body text */
@font-face {
  font-family: 'General Sans';
  font-style: normal;
  font-weight: 400;
  font-display: swap;
  src: url('/fonts/GeneralSans/GeneralSans-Regular.woff2') format('woff2'),
       url('/fonts/GeneralSans/GeneralSans-Regular.woff') format('woff');
}

@font-face {
  font-family: 'General Sans';
  font-style: normal;
  font-weight: 600;
  font-display: swap;
  src: url('/fonts/GeneralSans/GeneralSans-Medium.woff2') format('woff2'),
       url('/fonts/GeneralSans/GeneralSans-Medium.woff') format('woff');
}

/* SourceSerif4 - for textbook page body text */
@font-face {
  font-family: 'Source Serif 4';
  font-style: normal;
  font-weight: 400;
  font-display: swap;
  src: url('/fonts/SourceSerif4/SourceSerif4-Regular.woff2') format('woff2'),
       url('/fonts/SourceSerif4/SourceSerif4-Regular.woff') format('woff');
}

@font-face {
  font-family: 'Source Serif 4';
  font-style: normal;
  font-weight: 600;
  font-display: swap;
  src: url('/fonts/SourceSerif4/SourceSerif4-SemiBold.woff2') format('woff2'),
       url('/fonts/SourceSerif4/SourceSerif4-SemiBold.woff') format('woff');
}
```

## Step 3: Theme and Color Setup

### 3.1 Update Custom CSS
Update `src/css/custom.css` to include the indigo primary color and theme variables:

```css
/* Color variables */
:root {
  --ifm-color-primary: #6C3BAA; /* Indigo primary color */
  --ifm-color-primary-dark: #5a3291;
  --ifm-color-primary-darker: #552f89;
  --ifm-color-primary-darkest: #44266e;
  --ifm-color-primary-light: #7e4ecd;
  --ifm-color-primary-lighter: #8557d0;
  --ifm-color-primary-lightest: #976bd8;
}

/* Landing page typography */
.landing-page {
  --ifm-font-family-base: 'General Sans', system-ui, -apple-system, sans-serif;
  --ifm-heading-font-family: 'Archivo', system-ui, -apple-system, sans-serif;
}

/* Textbook page typography */
.textbook-page {
  --ifm-font-family-base: 'Source Serif 4', serif;
  --ifm-heading-font-family: 'Archivo', system-ui, -apple-system, sans-serif;
}

/* Dark mode variables */
[data-theme='dark'] {
  --ifm-color-primary: #b18cff;
  --ifm-color-primary-dark: #9267ff;
  --ifm-color-primary-darker: #8557d0;
  --ifm-color-primary-darkest: #6c3baa;
  --ifm-color-primary-light: #c9b0ff;
  --ifm-color-primary-lighter: #d8c9ff;
  --ifm-color-primary-lightest: #f0ebff;
}
```

## Step 4: Create Custom Components

### 4.1 Create Feature Card Component
Create `src/components/FeatureCard/index.js`:

```jsx
import React, { useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

export default function FeatureCard({ title, description, icon: Icon, link, color = '#6C3BAA' }) {
  const [isHovered, setIsHovered] = useState(false);

  return (
    <div
      className={clsx(styles.featureCard, {
        [styles.hovered]: isHovered
      })}
      style={{ borderLeft: `4px solid ${color}` }}
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
      onFocus={() => setIsHovered(true)}
      onBlur={() => setIsHovered(false)}
      tabIndex={0}
    >
      <div className={styles.iconContainer}>
        {Icon && <Icon className={styles.icon} />}
      </div>
      <h3 className={styles.title}>{title}</h3>
      <p className={styles.description}>{description}</p>
      {link && (
        <Link
          className={clsx('button button--primary', styles.linkButton)}
          to={link}
          style={{ backgroundColor: color, borderColor: color }}
        >
          Learn More
        </Link>
      )}
    </div>
  );
}
```

### 4.2 Create Feature Card Styles
Create `src/components/FeatureCard/styles.module.css`:

```css
.featureCard {
  background: var(--ifm-color-emphasis-100);
  padding: 2rem;
  border-radius: 8px;
  box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
  transition: all 0.3s ease;
  border-left: 4px solid var(--ifm-color-primary);
  height: 100%;
}

.featureCard:hover,
.featureCard:focus,
.hovered {
  transform: translateY(-10px);
  box-shadow: 0 12px 20px rgba(0, 0, 0, 0.15);
}

.iconContainer {
  margin-bottom: 1rem;
}

.icon {
  width: 48px;
  height: 48px;
  color: var(--ifm-color-primary);
}

.title {
  margin-top: 0;
  color: var(--ifm-heading-color);
  font-family: var(--ifm-heading-font-family);
}

.description {
  color: var(--ifm-color-emphasis-800);
  margin-bottom: 1.5rem;
}

.linkButton {
  text-decoration: none;
  padding: 0.75rem 1.5rem;
  display: inline-block;
  transition: all 0.3s ease;
}

.linkButton:hover {
  transform: scale(1.05);
}

/* Responsive design */
@media (max-width: 996px) {
  .featureCard {
    margin-bottom: 1.5rem;
  }

  .featureCard:hover,
  .featureCard:focus,
  .hovered {
    transform: none; /* Disable hover effect on mobile */
  }
}
```

## Step 5: Update Landing Page

### 5.1 Update Landing Page Component
Update `src/pages/index.js` to implement the new design:

```jsx
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import FeatureCard from '@site/src/components/FeatureCard';
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

function WhatYouWillLearn() {
  const learningPoints = [
    "Modern UI/UX design principles for educational content",
    "Docusaurus customization techniques",
    "Accessibility best practices",
    "Responsive design implementation"
  ];

  return (
    <section className={styles.learningSection}>
      <div className="container">
        <h2>What You Will Learn</h2>
        <ul className={styles.learningList}>
          {learningPoints.map((point, index) => (
            <li key={index} className={styles.learningItem}>
              <span className={styles.checkmark}>✓</span>
              {point}
            </li>
          ))}
        </ul>
      </div>
    </section>
  );
}

function FeaturedChapters() {
  const chapters = [
    {
      id: 'chapter-1',
      title: 'Introduction to Docusaurus',
      description: 'Learn the basics of Docusaurus and how to set up your first site',
      path: '/docs/intro'
    },
    {
      id: 'chapter-2',
      title: 'Customization Techniques',
      description: 'Advanced techniques for customizing your Docusaurus site',
      path: '/docs/customization'
    },
    {
      id: 'chapter-3',
      title: 'Performance Optimization',
      description: 'Best practices for optimizing your Docusaurus site performance',
      path: '/docs/performance'
    }
  ];

  return (
    <section className={styles.featuredChapters}>
      <div className="container">
        <h2>Featured Chapters</h2>
        <div className={styles.chaptersGrid}>
          {chapters.map((chapter) => (
            <div key={chapter.id} className={styles.chapterCard}>
              <h3>{chapter.title}</h3>
              <p>{chapter.description}</p>
              <Link
                className="button button--outline button--sm"
                to={chapter.path}
              >
                Read Chapter
              </Link>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  const featureCards = [
    {
      id: 'feature-1',
      title: 'Modern Design',
      description: 'Clean, contemporary UI with attention to typography and spacing',
      color: '#6C3BAA',
      link: '/docs/design-principles'
    },
    {
      id: 'feature-2',
      title: 'Accessibility First',
      description: 'Built with WCAG 2.1 AA compliance in mind for inclusive design',
      color: '#42c542',
      link: '/docs/accessibility'
    },
    {
      id: 'feature-3',
      title: 'Fully Responsive',
      description: 'Looks great on all devices from mobile to desktop',
      color: '#29adff',
      link: '/docs/responsive-design'
    }
  ];

  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Modern Docusaurus UI with improved typography and accessibility">
      <HomepageHeader />
      <main>
        {/* Feature Cards Section */}
        <section className={styles.featuresSection}>
          <div className="container">
            <h2>Key Features</h2>
            <div className={styles.featuresGrid}>
              {featureCards.map((card) => (
                <FeatureCard
                  key={card.id}
                  title={card.title}
                  description={card.description}
                  color={card.color}
                  link={card.link}
                />
              ))}
            </div>
          </div>
        </section>

        <WhatYouWillLearn />
        <FeaturedChapters />
      </main>
    </Layout>
  );
}
```

### 5.2 Update Landing Page Styles
Update `src/pages/index.module.css`:

```css
/* Hero Banner */
.heroBanner {
  padding: 4rem 0;
  text-align: center;
  position: relative;
  overflow: hidden;
  background: linear-gradient(135deg, #f5f7fa 0%, #e4edf5 100%);
}

[data-theme='dark'] .heroBanner {
  background: linear-gradient(135deg, #0d1117 0%, #161b22 100%);
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

.buttons a {
  transition: all 0.3s ease;
  transform: translateY(0);
}

.buttons a:hover {
  transform: translateY(-3px);
  box-shadow: 0 10px 20px rgba(0, 0, 0, 0.1);
}

/* Features Section */
.featuresSection {
  padding: 4rem 0;
}

.featuresSection h2 {
  text-align: center;
  margin-bottom: 3rem;
  font-family: 'Archivo', system-ui, -apple-system, sans-serif;
}

.featuresGrid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 2rem;
  max-width: 1200px;
  margin: 0 auto;
}

/* What You Will Learn Section */
.learningSection {
  padding: 4rem 0;
  background-color: var(--ifm-color-emphasis-100);
}

.learningSection h2 {
  text-align: center;
  margin-bottom: 2rem;
}

.learningList {
  max-width: 800px;
  margin: 0 auto;
  padding: 0;
  list-style: none;
}

.learningItem {
  display: flex;
  align-items: flex-start;
  margin-bottom: 1rem;
  padding: 1rem;
  background: white;
  border-radius: 8px;
  box-shadow: 0 2px 5px rgba(0, 0, 0, 0.05);
}

[data-theme='dark'] .learningItem {
  background: var(--ifm-color-emphasis-100);
}

.checkmark {
  display: inline-block;
  width: 24px;
  height: 24px;
  line-height: 24px;
  text-align: center;
  background-color: #6C3BAA;
  color: white;
  border-radius: 50%;
  margin-right: 1rem;
  flex-shrink: 0;
}

/* Featured Chapters Section */
.featuredChapters {
  padding: 4rem 0;
}

.featuredChapters h2 {
  text-align: center;
  margin-bottom: 3rem;
}

.chaptersGrid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 2rem;
  max-width: 1200px;
  margin: 0 auto;
}

.chapterCard {
  padding: 2rem;
  border-radius: 8px;
  box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
  border-left: 4px solid var(--ifm-color-primary);
  transition: all 0.3s ease;
}

.chapterCard:hover {
  transform: translateY(-5px);
  box-shadow: 0 12px 20px rgba(0, 0, 0, 0.15);
}

/* Responsive adjustments */
@media (max-width: 768px) {
  .buttons {
    flex-direction: column;
    gap: 1rem;
  }

  .featuresGrid,
  .chaptersGrid {
    grid-template-columns: 1fr;
  }

  .learningItem {
    flex-direction: column;
  }

  .checkmark {
    margin-bottom: 0.5rem;
  }
}
```

## Step 6: Update Docusaurus Configuration

### 6.1 Update docusaurus.config.js
Update the theme configuration to ensure proper dark mode behavior:

```javascript
// In docusaurus.config.js, ensure the colorMode is configured properly
themeConfig: {
  colorMode: {
    defaultMode: 'dark', // Set dark mode as default
    disableSwitch: false,
    respectPrefersColorScheme: true,
  },
  // ... other configurations
}
```

## Step 7: Update Textbook Page Typography

### 7.1 Create Textbook Page Wrapper
Create `src/components/TextbookPageWrapper/index.js`:

```jsx
import React from 'react';
import Layout from '@theme/Layout';

export default function TextbookPageWrapper(props) {
  return (
    <Layout {...props} className="textbook-page">
      {props.children}
    </Layout>
  );
}
```

## Step 8: Testing and Validation

### 8.1 Manual Testing Checklist
- [ ] Landing page displays all four required sections (FR-001)
- [ ] Typography is correct per page type (FR-002, FR-003)
- [ ] Dark mode is default, light mode toggle works (FR-004)
- [ ] Indigo primary color (#6C3BAA) is used throughout (FR-005)
- [ ] No emojis, only icons are used (FR-006)
- [ ] Contrast ratios meet WCAG 2.1 AA standards (FR-007)
- [ ] Navigation includes all required elements (FR-008)
- [ ] Footer has required sections (FR-009)
- [ ] Hover states are improved (FR-010)
- [ ] All existing content remains unchanged (FR-011)
- [ ] All interactive elements are keyboard accessible (FR-012)
- [ ] Responsive design works across devices (FR-013)

### 8.2 Performance Testing
```bash
npm run build
npm run serve
# Check page load times and Core Web Vitals
```

## Step 9: Build and Deploy

### 9.1 Build for Production
```bash
npm run build
```

### 9.2 Local Serving for Final Testing
```bash
npm run serve
```

The site will be available at http://localhost:3000 for final validation before deployment.