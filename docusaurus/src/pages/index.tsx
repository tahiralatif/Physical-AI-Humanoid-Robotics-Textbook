// import type { ReactNode } from 'react';
// import clsx from 'clsx';
// import Link from '@docusaurus/Link';
// import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
// import Layout from '@theme/Layout';
// import HomepageFeatures from '@site/src/components/HomepageFeatures';
// import Heading from '@theme/Heading';

// import styles from './index.module.css';

// function HomepageHeader() {
//   const { siteConfig } = useDocusaurusContext();
//   return (
//     <header className={styles.heroBanner}>
//       <div className="container">
//         <Heading as="h1" className={styles.heroTitle}>
//           {siteConfig.title}
//         </Heading>
//         <p className={styles.heroSubtitle}>
//           Master Physical AI & Humanoid Robotics in 13 Weeks
//         </p>
//         <p className={styles.heroSubtitle} style={{ fontSize: '1.125rem', marginTop: '0.5rem' }}>
//           From ROS2 Fundamentals to Building Real Humanoid Robots
//         </p>
//         <div className={styles.buttons}>
//           <Link
//             className={clsx('button', styles.primaryButton)}
//             to="/docs/introduction/overview">
//             🚀 Start Learning
//           </Link>
//           <Link
//             className={clsx('button', styles.secondaryButton)}
//             to="/docs/introduction/course-structure">
//             📚 View Course Structure
//           </Link>
//         </div>
//         {/* Premium Robot Hero Illustration */}
//         <div className={styles.heroIllustration}>
//           <img
//             src="/img/hero-robot.png"
//             alt="AI Humanoid Robot - Physical AI and Robotics"
//           />
//         </div>
//       </div>
//     </header>
//   );
// }

// export default function Home(): ReactNode {
//   const { siteConfig } = useDocusaurusContext();
//   return (
//     <Layout
//       title="Home"
//       description="A comprehensive 13-week interactive textbook for learning Physical AI and Humanoid Robotics. Master ROS2, PyTorch, Isaac Sim, and build real robots.">
//       <HomepageHeader />
//       <main>
//         <HomepageFeatures />
//       </main>
//     </Layout>
//   );
// }


import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Docusaurus Tutorial - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}