import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useTheme } from '../ThemeSystem';
import ThemeToggleButton from '../ThemeToggleButton';
import styles from './styles.module.css';

const NavigationBar = ({ className = '', ...props }) => {
  const { siteConfig } = useDocusaurusContext();
  const { theme } = useTheme();

  return (
    <nav className={clsx(styles.navigationBar, className)} {...props}>
      <div className={styles.navbarItems}>
        <Link className={styles.navbarBrand} to="/">
          <span className={styles.navbarTitle}>{siteConfig.title}</span>
        </Link>
        <Link className={clsx(styles.navbarLink)} to="/docs/intro">
          Textbook
        </Link>
      </div>
      <div className={clsx(styles.navbarItems, styles.navbarItemsRight)}>
        <div className={styles.navbarItem}>
          <input
            type="text"
            placeholder="Search..."
            className={styles.navbarSearchInput}
            aria-label="Search"
          />
        </div>
        <div className={styles.navbarItem}>
          <ThemeToggleButton className={styles.themeToggleButton} />
        </div>
      </div>
    </nav>
  );
};

export default NavigationBar;