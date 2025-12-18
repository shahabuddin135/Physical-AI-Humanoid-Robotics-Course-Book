import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Build robots that actually work. Dark humor included, sanity not guaranteed.',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  url: 'https://physical-ai-book.dev',
  baseUrl: '/',

  organizationName: 'physical-ai',
  projectName: 'humanoid-robotics-book',

  onBrokenLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl: 'https://github.com/physical-ai/humanoid-robotics-book/edit/main/',
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          editUrl: 'https://github.com/physical-ai/humanoid-robotics-book/edit/main/',
          blogTitle: 'Robot Lab Notes',
          blogDescription: 'Updates, tutorials, and robot mishaps from the Physical AI team.',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/social-card.jpg',
    colorMode: {
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: '',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo-light.svg',
        srcDark: 'img/logo-dark.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'bookSidebar',
          position: 'left',
          label: 'The Book',
        },
        {
          to: '/blog',
          label: 'Lab Notes',
          position: 'left',
        },
        {
          to: '/chatbot',
          label: 'Ask ROBO-SAGE',
          position: 'left',
        },
        {
          href: 'https://github.com/physical-ai/humanoid-robotics-book',
          position: 'right',
          className: 'header-github-link',
          'aria-label': 'GitHub repository',
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
              label: 'Start Reading',
              to: '/docs',
            },
            {
              label: 'Module 1: ROS 2',
              to: '/docs/module-1-ros2',
            },
            {
              label: 'Module 2: Simulation',
              to: '/docs/module-2-simulation',
            },
            {
              label: 'Module 3: NVIDIA Isaac',
              to: '/docs/module-3-isaac',
            },
            {
              label: 'Module 4: VLA',
              to: '/docs/module-4-vla',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Discord',
              href: 'https://discord.gg/physical-ai',
            },
            {
              label: 'Twitter / X',
              href: 'https://x.com/physicalai',
            },
            {
              label: 'GitHub Discussions',
              href: 'https://github.com/physical-ai/humanoid-robotics-book/discussions',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Lab Notes (Blog)',
              to: '/blog',
            },
            {
              label: 'Ask ROBO-SAGE',
              to: '/chatbot',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/physical-ai/humanoid-robotics-book',
            },
          ],
        },
      ],
      copyright: `© ${new Date().getFullYear()} Physical AI. Built with caffeine, existential dread, and Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml', 'json', 'cpp'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
