import clsx from "clsx";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import FeatureCard from "@site/src/components/FeatureCard";
import ResponsiveCardGrid from "@site/src/components/ResponsiveCardGrid";
import LearningOutcomes from "@site/src/components/LearningOutcomes";
import FeaturedChapters from "@site/src/components/FeaturedChapters";
import AccessibleButton from "@site/src/components/AccessibleButton";
import { ThemeProvider } from "@site/src/components/ThemeSystem";
import TypographyWrapper from "@site/src/components/TypographySystem";

import Heading from "@theme/Heading";
import styles from "./index.module.css";
import { useEffect, useState } from "react";

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  const [titleText, setTitleText] = useState("");
  const fullTitle = siteConfig.title;

  // Typewriter effect for title
  useEffect(() => {
    let index = 0;
    const timer = setInterval(() => {
      if (index <= fullTitle.length) {
        setTitleText(fullTitle.slice(0, index));
        index++;
      } else {
        clearInterval(timer);
      }
    }, 100);

    return () => clearInterval(timer);
  }, [fullTitle]);

  return (
    <header className={clsx("hero hero--primary", styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <TypographyWrapper variant="heading" pageType="landing">
            <Heading as="h1" className="hero__title" data-text={titleText}>
              {titleText}
              <span className={styles.cursor}>|</span>
            </Heading>
          </TypographyWrapper>
          <TypographyWrapper variant="body" pageType="landing">
            <p className="hero__subtitle">{siteConfig.tagline}</p>
          </TypographyWrapper>
          <div className={styles.buttons}>
            <AccessibleButton
              variant="primary"
              href="/docs/intro"
              icon="arrow"
              className={styles.primaryButton}
            >
              Get Started
            </AccessibleButton>
            <AccessibleButton
              variant="secondary"
              href="/docs/tutorial-basics/congratulations"
              icon="book"
              className={styles.secondaryButton}
            >
              View Examples
            </AccessibleButton>
          </div>
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
    "Responsive design implementation",
  ];

  return (
    <section className={styles.learningSection}>
      <div className="container">
        <TypographyWrapper variant="heading" pageType="landing">
          <h2>What You Will Learn</h2>
        </TypographyWrapper>
        <LearningOutcomes outcomes={learningPoints} />
      </div>
    </section>
  );
}

function FeaturedChaptersSection() {
  const chapters = [
    {
      id: "chapter-1",
      title: "Introduction to Docusaurus",
      description:
        "Learn the basics of Docusaurus and how to set up your first site",
      path: "/docs/intro",
    },
    {
      id: "chapter-2",
      title: "Customization Techniques",
      description: "Advanced techniques for customizing your Docusaurus site",
      path: "/docs/tutorial-basics/create-a-document",
    },
    {
      id: "chapter-3",
      title: "Performance Optimization",
      description:
        "Best practices for optimizing your Docusaurus site performance",
      path: "/docs/tutorial-extras/manage-docs-versions",
    },
  ];

  return (
    <section className={styles.featuredChapters}>
      <div className="container">
        <TypographyWrapper variant="heading" pageType="landing">
          <h2>Featured Chapters</h2>
        </TypographyWrapper>
        <FeaturedChapters chapters={chapters} />
      </div>
    </section>
  );
}

function ScrollToTop() {
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    const toggleVisibility = () => {
      if (window.pageYOffset > 300) {
        setIsVisible(true);
      } else {
        setIsVisible(false);
      }
    };

    window.addEventListener("scroll", toggleVisibility);
    return () => window.removeEventListener("scroll", toggleVisibility);
  }, []);

  const scrollToTop = () => {
    window.scrollTo({
      top: 0,
      behavior: "smooth",
    });
  };

  return (
    <button
      className={clsx(styles.backToTop, isVisible && styles.visible)}
      onClick={scrollToTop}
      aria-label="Scroll to top"
    >
      ↑
    </button>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  const featureCards = [
    {
      id: "feature-1",
      title: "Modern Design",
      description:
        "Clean, contemporary UI with attention to typography and spacing",
      color: "#6C3BAA",
      link: "/docs/intro",
    },
    {
      id: "feature-2",
      title: "Accessibility First",
      description:
        "Built with WCAG 2.1 AA compliance in mind for inclusive design",
      color: "#10b981",
      link: "/docs/tutorial-basics/create-a-document",
    },
    {
      id: "feature-3",
      title: "Fully Responsive",
      description: "Looks great on all devices from mobile to desktop",
      color: "#3b82f6",
      link: "/docs/tutorial-basics/create-a-page",
    },
  ];

  return (
    <ThemeProvider>
      <Layout
        title={`${siteConfig.title}`}
        description="Modern Docusaurus UI with improved typography and accessibility"
      >
        <HomepageHeader />
        <main>
          {/* Feature Cards Section */}
          <section className={styles.featuresSection}>
            <div className="container">
              <h2>Key Features</h2>
              <ResponsiveCardGrid className={styles.featuresGrid}>
                {featureCards.map((card) => (
                  <FeatureCard
                    key={card.id}
                    title={card.title}
                    description={card.description}
                    color={card.color}
                    link={card.link}
                  />
                ))}
              </ResponsiveCardGrid>
            </div>
          </section>

          <WhatYouWillLearn />
          <FeaturedChaptersSection />
        </main>
        <ScrollToTop />
      </Layout>
    </ThemeProvider>
  );
}
