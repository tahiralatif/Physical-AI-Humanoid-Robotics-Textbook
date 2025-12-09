import type { ReactNode } from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: '13-Week Structured Curriculum',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Master Physical AI from fundamentals to advanced applications.
        Progress through ROS2, Digital Twins, Neural Networks, and Humanoid Robotics
        with a carefully designed learning path.
      </>
    ),
  },
  {
    title: 'Hands-On Projects',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Build real robots and AI systems. Work with TurtleBot, Gazebo simulations,
        PyTorch models, and NVIDIA Isaac Sim. Every week includes practical
        exercises and real-world applications.
      </>
    ),
  },
  {
    title: 'Industry-Ready Skills',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Learn cutting-edge tools used by robotics engineers worldwide: ROS2,
        PyTorch, Isaac Sim, Gazebo, and more. Graduate ready to work on
        humanoid robots and physical AI systems.
      </>
    ),
  },
];

function Feature({ title, Svg, description }: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className={clsx('text--center', styles.featureCard)}>
        <Svg className={styles.featureSvg} role="img" />
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
