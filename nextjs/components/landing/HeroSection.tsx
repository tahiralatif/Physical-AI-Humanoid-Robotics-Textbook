"use client"

import { motion } from "framer-motion"
import { Button } from "@/components/ui/button"
import { ArrowRight, Sparkles } from "lucide-react"
import Link from "next/link"

export function HeroSection() {
    return (
        <section className="relative min-h-screen flex items-center justify-center overflow-hidden pt-16">
            {/* Animated Gradient Background */}
            <div className="absolute inset-0 bg-gradient-radial opacity-30" />
            <div className="absolute inset-0 bg-gradient-to-b from-transparent via-background/50 to-background" />

            {/* Floating Orbs */}
            <motion.div
                className="absolute top-1/4 left-1/4 w-96 h-96 bg-gradient-primary rounded-full blur-3xl opacity-20"
                animate={{
                    scale: [1, 1.2, 1],
                    opacity: [0.2, 0.3, 0.2],
                }}
                transition={{
                    duration: 8,
                    repeat: Infinity,
                    ease: "easeInOut",
                }}
            />
            <motion.div
                className="absolute bottom-1/4 right-1/4 w-96 h-96 bg-gradient-secondary rounded-full blur-3xl opacity-20"
                animate={{
                    scale: [1.2, 1, 1.2],
                    opacity: [0.3, 0.2, 0.3],
                }}
                transition={{
                    duration: 10,
                    repeat: Infinity,
                    ease: "easeInOut",
                }}
            />

            {/* Content */}
            <div className="relative z-10 max-w-6xl mx-auto px-4 sm:px-6 lg:px-8 text-center">
                <motion.div
                    initial={{ opacity: 0, y: 20 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ duration: 0.6 }}
                    className="flex items-center justify-center gap-2 mb-6"
                >
                    <Sparkles className="w-5 h-5 text-accent" />
                    <span className="text-sm font-medium text-accent uppercase tracking-wide">
                        13-Week Interactive Course
                    </span>
                </motion.div>

                <motion.h1
                    initial={{ opacity: 0, y: 20 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ duration: 0.6, delay: 0.1 }}
                    className="text-5xl md:text-7xl font-bold font-[family-name:var(--font-heading)] mb-6 leading-tight"
                >
                    Master{" "}
                    <span className="text-gradient-primary">Physical AI</span>
                    {" "}&{" "}
                    <span className="text-gradient-secondary">Humanoid Robotics</span>
                </motion.h1>

                <motion.p
                    initial={{ opacity: 0, y: 20 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ duration: 0.6, delay: 0.2 }}
                    className="text-xl md:text-2xl text-muted max-w-3xl mx-auto mb-12"
                >
                    Build real robots from scratch. Learn ROS2, Digital Twins, Neural Networks, and deploy AI on physical hardware.
                </motion.p>

                <motion.div
                    initial={{ opacity: 0, y: 20 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ duration: 0.6, delay: 0.3 }}
                    className="flex flex-col sm:flex-row gap-4 justify-center"
                >
                    <Link href="/book/intro">
                        <Button variant="primary" size="lg" className="group">
                            Start Learning
                            <ArrowRight className="w-5 h-5 group-hover:translate-x-1 transition-transform" />
                        </Button>
                    </Link>
                    <Link href="/book">
                        <Button variant="outline" size="lg">
                            View Course Modules
                        </Button>
                    </Link>
                </motion.div>

                {/* Stats */}
                <motion.div
                    initial={{ opacity: 0 }}
                    animate={{ opacity: 1 }}
                    transition={{ duration: 0.6, delay: 0.5 }}
                    className="mt-16 grid grid-cols-3 gap-8 max-w-2xl mx-auto"
                >
                    {[
                        { label: "Weeks", value: "13" },
                        { label: "Modules", value: "4" },
                        { label: "Projects", value: "10+" },
                    ].map((stat, i) => (
                        <div key={i} className="text-center">
                            <div className="text-4xl font-bold text-gradient-primary mb-2">
                                {stat.value}
                            </div>
                            <div className="text-sm text-muted uppercase tracking-wide">
                                {stat.label}
                            </div>
                        </div>
                    ))}
                </motion.div>
            </div>
        </section>
    )
}
