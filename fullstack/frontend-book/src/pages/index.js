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
              Start Learning
            </AccessibleButton>
            <AccessibleButton
              variant="secondary"
              href="/docs/tutorial-basics/congratulations"
              icon="book"
              className={styles.secondaryButton}
            >
              View Curriculum Roadmap
            </AccessibleButton>
          </div>
        </div>
      </div>
    </header>
  );
}

function WhatYouWillLearn() {
  const learningPoints = [
    "Foundations of Physical AI and embodied intelligence",
    "Humanoid robot kinematics and physical structure",
    "Digital twins using Gazebo and Unity",
    "Perception systems: cameras, LiDAR, IMUs, and sensor fusion",
    "AI robot brains with NVIDIA Isaac and Nav2",
    "Vision–Language–Action pipelines for humanoid autonomy",
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
      title: "Foundations of Physical AI",
      description:
        "Understand embodied intelligence and why humanoid robots require a new AI paradigm.",
      path: "/docs/intro",
    },
    {
      id: "chapter-2",
      title: "Digital Twins & Simulation",
      description: "Build realistic robot simulations with physics, sensors, and environments.",
      path: "/docs/module-3/",
    },
    {
      id: "chapter-3",
      title: "Perception, AI Brain & Autonomy",
      description:
        "Teach robots to see, navigate, reason, and act in the real world.",
      path: "/docs/module-6/",
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
      title: "Spec-Driven Learning",
      description:
        "Every module is defined by clear specifications, learning goals, and success criteria — just like real-world engineering systems.",
      color: "#6C3BAA",
      link: "/docs/intro",
    },
    {
      id: "feature-2",
      title: "End-to-End Humanoid Pipeline",
      description:
        "Learn the full autonomy stack: kinematics, simulation, perception, AI navigation, and language-driven action.",
      color: "#10b981",
      link: "/docs/module-1/",
    },
    {
      id: "feature-3",
      title: "Simulation-to-Reality Focus",
      description: "From Gazebo and Unity to NVIDIA Isaac and ROS 2 — bridge virtual robots to real humanoid systems.",
      color: "#3b82f6",
      link: "/docs/module-3/",
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
