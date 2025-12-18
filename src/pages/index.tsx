import type { ReactNode } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

// Hero Section
function HeroSection() {
  return (
    <section className={styles.hero}>
      <div className={styles.heroContent}>
        <p className={styles.heroLabel}>PHYSICAL AI CURRICULUM</p>
        <Heading as="h1" className={styles.heroTitle}>
          Build robots that
          <br />
          <span className={styles.heroAccent}>actually work</span>
        </Heading>
        <p className={styles.heroSubtitle}>
          Master ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action models. 
          <br />
          Dark humor included. Sanity not guaranteed.
        </p>
        <div className={styles.heroCta}>
          <Link className={clsx('button button--primary button--lg', styles.ctaPrimary)} to="/docs">
            Start Learning
          </Link>
          <Link className={clsx('button button--secondary button--lg', styles.ctaSecondary)} to="/chatbot">
            Ask ROBO-SAGE
          </Link>
        </div>
      </div>
      <div className={styles.heroVisual}>
        <div className={styles.robotIllustration}>
          <svg viewBox="0 0 400 400" fill="none" xmlns="http://www.w3.org/2000/svg" className={styles.robotSvg}>
            {/* Robot Head */}
            <rect x="125" y="80" width="150" height="120" rx="20" className={styles.robotBody} />
            {/* Eyes */}
            <circle cx="175" cy="130" r="15" className={styles.robotEye} />
            <circle cx="225" cy="130" r="15" className={styles.robotEye} />
            <circle cx="175" cy="130" r="8" className={styles.robotPupil} />
            <circle cx="225" cy="130" r="8" className={styles.robotPupil} />
            {/* Mouth */}
            <rect x="160" y="165" width="80" height="15" rx="5" className={styles.robotMouth} />
            {/* Antenna */}
            <line x1="200" y1="80" x2="200" y2="50" className={styles.robotAntenna} />
            <circle cx="200" cy="45" r="8" className={styles.robotAntennaTop} />
            {/* Body */}
            <rect x="140" y="210" width="120" height="100" rx="15" className={styles.robotBody} />
            {/* Chest Panel */}
            <rect x="165" y="230" width="70" height="60" rx="8" className={styles.robotPanel} />
            {/* Arms */}
            <rect x="90" y="220" width="40" height="80" rx="10" className={styles.robotArm} />
            <rect x="270" y="220" width="40" height="80" rx="10" className={styles.robotArm} />
            {/* Legs */}
            <rect x="155" y="320" width="35" height="60" rx="8" className={styles.robotLeg} />
            <rect x="210" y="320" width="35" height="60" rx="8" className={styles.robotLeg} />
            {/* Decorative elements */}
            <circle cx="200" cy="260" r="20" className={styles.robotCore} />
          </svg>
          {/* Floating elements */}
          <div className={clsx(styles.floatingElement, styles.float1)}>ROS 2</div>
          <div className={clsx(styles.floatingElement, styles.float2)}>Isaac</div>
          <div className={clsx(styles.floatingElement, styles.float3)}>Gazebo</div>
        </div>
      </div>
    </section>
  );
}

// Module Card Component
interface ModuleCardProps {
  number: string;
  title: string;
  subtitle: string;
  description: string;
  icon: string;
  color: string;
  link: string;
}

function ModuleCard({ number, title, subtitle, description, icon, color, link }: ModuleCardProps) {
  return (
    <Link to={link} className={styles.moduleCard}>
      <div className={styles.moduleNumber} style={{ color }}>
        {number}
      </div>
      <div className={styles.moduleIcon}>{icon}</div>
      <h3 className={styles.moduleTitle}>{title}</h3>
      <p className={styles.moduleSubtitle}>{subtitle}</p>
      <p className={styles.moduleDescription}>{description}</p>
      <div className={styles.moduleArrow}>→</div>
    </Link>
  );
}

// Modules Section
function ModulesSection() {
  const modules: ModuleCardProps[] = [
    {
      number: '01',
      title: 'ROS 2',
      subtitle: 'The Robotic Nervous System',
      description: 'Nodes, Topics, Services, and why your robot needs therapy (a.k.a. proper middleware).',
      icon: '🧠',
      color: '#2563EB',
      link: '/docs/module-1-ros2',
    },
    {
      number: '02',
      title: 'Simulation',
      subtitle: 'The Digital Twin',
      description: 'Gazebo & Unity: Where your robot can fail spectacularly without breaking anything expensive.',
      icon: '🌍',
      color: '#10B981',
      link: '/docs/module-2-simulation',
    },
    {
      number: '03',
      title: 'NVIDIA Isaac',
      subtitle: 'The AI-Robot Brain',
      description: 'GPU-powered perception and navigation. Warning: May cause CUDA dependency addiction.',
      icon: '⚡',
      color: '#76B900',
      link: '/docs/module-3-isaac',
    },
    {
      number: '04',
      title: 'VLA',
      subtitle: 'Vision-Language-Action',
      description: 'When LLMs meet robotics. "Clean the room" → robot actually cleans (hopefully).',
      icon: '🗣️',
      color: '#8B5CF6',
      link: '/docs/module-4-vla',
    },
  ];

  return (
    <section className={styles.modules}>
      <div className={styles.sectionHeader}>
        <p className={styles.sectionLabel}>CURRICULUM</p>
        <Heading as="h2" className={styles.sectionTitle}>
          Four modules to robot mastery
        </Heading>
        <p className={styles.sectionSubtitle}>
          From middleware madness to autonomous humanoids. Each module is designed to be 
          digestible, practical, and occasionally funny.
        </p>
      </div>
      <div className={styles.moduleGrid}>
        {modules.map((module, idx) => (
          <ModuleCard key={idx} {...module} />
        ))}
      </div>
    </section>
  );
}

// Features Section
function FeaturesSection() {
  const features = [
    {
      icon: '📖',
      title: 'Actually Readable',
      description: 'No academic jargon. Just clear explanations with the occasional robot pun.',
    },
    {
      icon: '🤖',
      title: 'RAG-Powered Chatbot',
      description: 'ROBO-SAGE answers your questions using the book content. It\'s like Clippy, but useful.',
    },
    {
      icon: '🧪',
      title: 'Hands-On Projects',
      description: 'Every module ends with a project. By module 4, you\'ll have an autonomous humanoid.',
    },
    {
      icon: '😈',
      title: 'Dark Humor',
      description: 'Because debugging ROS 2 without humor is just masochism.',
    },
    {
      icon: '🎯',
      title: 'Production-Ready',
      description: 'Real-world patterns from actual robot deployments. Not just toy examples.',
    },
    {
      icon: '🔥',
      title: 'Open Source',
      description: 'All code, all examples, all available. Fork it, break it, fix it, ship it.',
    },
  ];

  return (
    <section className={styles.features}>
      <div className={styles.sectionHeader}>
        <p className={styles.sectionLabel}>WHY THIS BOOK?</p>
        <Heading as="h2" className={styles.sectionTitle}>
          Learning robotics shouldn't suck
        </Heading>
      </div>
      <div className={styles.featureGrid}>
        {features.map((feature, idx) => (
          <div key={idx} className={styles.featureCard}>
            <span className={styles.featureIcon}>{feature.icon}</span>
            <h3 className={styles.featureTitle}>{feature.title}</h3>
            <p className={styles.featureDescription}>{feature.description}</p>
          </div>
        ))}
      </div>
    </section>
  );
}

// Tech Stack Section
function TechStackSection() {
  const technologies = [
    { name: 'ROS 2 Humble', category: 'Middleware' },
    { name: 'Gazebo', category: 'Simulation' },
    { name: 'Unity', category: 'Simulation' },
    { name: 'NVIDIA Isaac Sim', category: 'AI Platform' },
    { name: 'Isaac ROS', category: 'Perception' },
    { name: 'Nav2', category: 'Navigation' },
    { name: 'OpenAI Whisper', category: 'Speech' },
    { name: 'Vision Transformers', category: 'Computer Vision' },
  ];

  return (
    <section className={styles.techStack}>
      <div className={styles.sectionHeader}>
        <p className={styles.sectionLabel}>TECH STACK</p>
        <Heading as="h2" className={styles.sectionTitle}>
          Industry-standard tools
        </Heading>
        <p className={styles.sectionSubtitle}>
          The same technologies used by Boston Dynamics, Tesla Bot, and Figure AI.
        </p>
      </div>
      <div className={styles.techGrid}>
        {technologies.map((tech, idx) => (
          <div key={idx} className={styles.techItem}>
            <span className={styles.techName}>{tech.name}</span>
            <span className={styles.techCategory}>{tech.category}</span>
          </div>
        ))}
      </div>
    </section>
  );
}

// CTA Section
function CtaSection() {
  return (
    <section className={styles.cta}>
      <div className={styles.ctaContent}>
        <Heading as="h2" className={styles.ctaTitle}>
          Ready to build the future?
        </Heading>
        <p className={styles.ctaSubtitle}>
          Start with Module 1, or jump straight to whatever looks interesting. 
          We won't judge. Much.
        </p>
        <div className={styles.ctaButtons}>
          <Link className="button button--primary button--lg" to="/docs">
            Start Reading →
          </Link>
          <Link className="button button--secondary button--lg" to="/chatbot">
            Chat with ROBO-SAGE
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Master Physical AI and build humanoid robots with ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action models. Dark humor included.">
      <main className={styles.main}>
        <HeroSection />
        <ModulesSection />
        <FeaturesSection />
        <TechStackSection />
        <CtaSection />
      </main>
    </Layout>
  );
}
