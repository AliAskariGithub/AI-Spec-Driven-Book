import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

const FooterSections = ({ sections = [], copyrightText = '', className = '', ...props }) => {
  return (
    <footer className={clsx(styles.footer, className)} {...props}>
      <div className="container">
        <div className={styles.footerRow}>
          {sections.map((section, index) => (
            <div key={index} className={styles.footerCol}>
              <h4 className={styles.footerTitle}>{section.title}</h4>
              <ul className={styles.footerItems}>
                {section.items.map((item, itemIndex) => (
                  <li key={itemIndex} className={styles.footerItem}>
                    <Link className={styles.footerLinkItem} to={item.href}>
                      {item.label}
                    </Link>
                  </li>
                ))}
              </ul>
            </div>
          ))}
        </div>
        <div className={styles.footerBottom}>
          <div className={styles.footerCopyright}>
            {copyrightText}
          </div>
        </div>
      </div>
    </footer>
  );
};

export default FooterSections;