"use client"

import { motion } from "framer-motion"
import { Card, CardHeader, CardTitle, CardDescription, CardContent } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Cpu, Box, Brain, Bot } from "lucide-react"

const modules = [
    {
        title: "Module 1: ROS2 Fundamentals",
        description: "Master Robot Operating System 2, nodes, topics, and services. Build your first autonomous robot.",
        weeks: "3 weeks",
        icon: Cpu,
        gradient: "from-purple-500 to-pink-500",
    },
    {
        title: "Module 2: Digital Twins",
        description: "Create virtual representations of physical robots using Gazebo and URDF.",
        weeks: "3 weeks",
        icon: Box,
        gradient: "from-blue-500 to-cyan-500",
    },
    {
        title: "Module 3: Neural Networks",
        description: "Implement deep learning models with PyTorch for robot perception and decision-making.",
        weeks: "3 weeks",
        icon: Brain,
        gradient: "from-pink-500 to-rose-500",
    },
    {
        title: "Module 4: Humanoid Robotics",
        description: "Build and program humanoid robots with advanced kinematics and motion planning.",
        weeks: "4 weeks",
        icon: Bot,
        gradient: "from-violet-500 to-purple-500",
    },
]

export function ModuleCards() {
    return (
        <section className="py-24 px-4 sm:px-6 lg:px-8">
            <div className="max-w-7xl mx-auto">
                <motion.div
                    initial={{ opacity: 0, y: 20 }}
                    whileInView={{ opacity: 1, y: 0 }}
                    viewport={{ once: true }}
                    transition={{ duration: 0.6 }}
                    className="text-center mb-16"
                >
                    <h2 className="text-4xl md:text-5xl font-bold font-[family-name:var(--font-heading)] mb-4">
                        Course <span className="text-gradient-primary">Modules</span>
                    </h2>
                    <p className="text-xl text-muted max-w-2xl mx-auto">
                        A structured path from fundamentals to advanced humanoid robotics
                    </p>
                </motion.div>

                <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
                    {modules.map((module, index) => {
                        const Icon = module.icon
                        return (
                            <motion.div
                                key={index}
                                initial={{ opacity: 0, y: 20 }}
                                whileInView={{ opacity: 1, y: 0 }}
                                viewport={{ once: true }}
                                transition={{ duration: 0.5, delay: index * 0.1 }}
                            >
                                <Card className="h-full hover:shadow-2xl">
                                    <CardHeader>
                                        <div className="flex items-start justify-between mb-4">
                                            <div className={`p-3 rounded-xl bg-gradient-to-br ${module.gradient}`}>
                                                <Icon className="w-6 h-6 text-white" />
                                            </div>
                                            <Badge variant="secondary">{module.weeks}</Badge>
                                        </div>
                                        <CardTitle>{module.title}</CardTitle>
                                        <CardDescription className="text-base mt-2">
                                            {module.description}
                                        </CardDescription>
                                    </CardHeader>
                                </Card>
                            </motion.div>
                        )
                    })}
                </div>
            </div>
        </section>
    )
}
