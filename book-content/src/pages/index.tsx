import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';

// Feature data - easy to update
const features = [
  {
    title: 'Comprehensive Curriculum',
    description: 'From the fundamentals of ROS 2 to advanced autonomous systems, our textbook provides a complete learning path.',
  },
  {
    title: 'Hands-On Projects',
    description: 'Build and simulate a humanoid robot, applying concepts from each chapter to see your learning in action.',
  },
  {
    title: 'AI-Native Approach',
    description: 'Explore modern AI techniques, including Large Language Models and Reinforcement Learning, tailored for robotics.',
  },
];

// Feature card component
function FeatureCard({title, description}: {title: string; description: string}) {
  return (
    <div className="feature-card">
      <h3 className="feature-card__title">{title}</h3>
      <p className="feature-card__description">{description}</p>
    </div>
  );
}

// Homepage hero section
function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className="hero-banner">
      <div className="container">
        <Heading as="h1" className="hero-banner__title">
          {siteConfig.title}
        </Heading>
        <p className="hero-banner__subtitle">{siteConfig.tagline}</p>
        <div className="hero-banner__buttons">
          <Link
            className="button button--primary button--lg"
            to="/docs/chapter1">
            Get Started
          </Link>
        </div>
      </div>
    </header>
  );
}

// Full homepage component
export default function Home(): React.ReactElement {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Home"
      description="A textbook on Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <section className="features-section">
          <div className="container">
            <div className="features-grid">
              {features.map((feature, idx) => (
                <FeatureCard key={idx} {...feature} />
              ))}
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}