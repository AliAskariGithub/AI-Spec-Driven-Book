// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'AI-Spec Driven Book',
  tagline: 'The Robotic Nervous System - ROS 2 for AI/CS Students',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://ai-spec-driven-book-six.vercel.app',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'AliAskariGithub', // Usually your GitHub org/user name.
  projectName: 'AI-Spec-Driven-Book', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenAnchors: 'ignore',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
        htmlLang: 'ur-PK',
      },
    },
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      colorMode: {
        defaultMode: 'dark', // Set dark mode as default
        respectPrefersColorScheme: true,
      },
      metadata: [
        {name: 'keywords', content: 'robotics, ROS 2, AI, computer science, education, middleware, simulation, NVIDIA Isaac, Gazebo, Unity'},
        {name: 'author', content: 'AI-Spec Driven Book Team'},
        {name: 'robots', content: 'index, follow'},
        {name: 'theme-color', content: '#2e8555'},
      ],
      navbar: {
        title: 'AI-Spec Driven Book',
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            type: 'search',
            position: 'right',
          },
          {
            href: 'https://github.com/AliAskariGithub',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Learn',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },
              {
                label: 'The Robotic Nervous System',
                to: '/docs/module-1/',
              },
              {
                label: 'The Digital Twin',
                to: '/docs/module-2/',
              },
            ],
          },
          {
            title: 'Social Links',
            items: [
              {
                label: 'Facebook',
                href: 'https://www.facebook.com/NexuGem',
              },
              {
                label: 'Instagram',
                href: 'https://www.instagram.com/nexugem/',
              },
              {
                label: 'X',
                href: 'https://x.com/Syed_Ali_Askari',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/AliAskariGithub',
              },
              {
                label: 'Documentation',
                to: '/docs/intro',
              },
              {
                label: 'Tutorials',
                to: '/docs/tutorial-basics/congratulations',
              },
            ],
          },
          {
            title: 'About',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/AliAskariGithub',
              },
              {
                label: 'Docusaurus',
                href: 'https://docusaurus.io/',
              },
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/docusaurus',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} AI-Spec Driven Book | Built by Ali Askari.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;
