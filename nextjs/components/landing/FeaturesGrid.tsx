"use client"

import { motion } from "framer-motion"
import { Wrench, Code2, Rocket } from "lucide-react"

const features = [
    {
        title: "Hands-On Projects",
        description: "Build real robots from TurtleBot basics to complex humanoid systems. Every week includes practical exercises.",
        icon: Wrench,
    },
    {
        title: "Industry Tools",
        description: "Master ROS2, PyTorch, Gazebo, NVIDIA Isaac Sim, and professional robotics development workflows.",
        icon: Code2,
    },
    {
        title: "Real Robots",
        description: "Deploy AI on physical hardware. Learn kinematics, motion planning, and autonomous navigation.",
        icon: Rocket,
    },
]

export function FeaturesGrid() {
    return (
        <section className="py-24 px-4 sm:px-6 lg:px-8 bg-gradient-to-b from-background via-muted/5 to-background">
            <div className="max-w-7xl mx-auto">
                <motion.div
                    initial={{ opacity: 0, y: 20 }}
                    whileInView={{ opacity: 1, y: 0 }}
                    viewport={{ once: true }}
                    transition={{ duration: 0.6 }}
                    className="text-center mb-16"
                >
                    <h2 className="text-4xl md:text-5xl font-bold font-[family-name:var(--font-heading)] mb-4">
                        Why <span className="text-gradient-secondary">This Course?</span>
                    </h2>
                    <p className="text-xl text-muted max-w-2xl mx-auto">
                        Industry-ready skills for the future of robotics
                    </p>
                </motion.div>

                <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
                    {features.map((feature, index) => {
                        const Icon = feature.icon
                        return (
                            <motion.div
                                key={index}
                                initial={{ opacity: 0, y: 20 }}
                                whileInView={{ opacity: 1, y: 0 }}
                                viewport={{ once: true }}
                                transition={{ duration: 0.5, delay: index * 0.1 }}
                                className="glass rounded-2xl p-8 hover-lift"
                            >
                                <div className="mb-4">
                                    <div className="inline-flex p-3 rounded-xl bg-gradient-primary">
                                        <Icon className="w-6 h-6 text-white" />
                                    </div>
                                </div>
                                <h3 className="text-2xl font-bold font-[family-name:var(--font-heading)] mb-3">
                                    {feature.title}
                                </h3>
                                <p className="text-muted leading-relaxed">
                                    {feature.description}
                                </p>
                            </motion.div>
                        )
                    })}
                </div>
            </div>
        </section>
    )
}
