"use client";

import { authClient } from "@/lib/auth-client";
import { useRouter } from "next/navigation";
import { useEffect, useState } from "react";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { LogOut, User, BookOpen, Activity, Link as LinkIcon, Bot, Cpu, Box, Brain } from "lucide-react";
import Link from "next/link";

export default function DashboardPage() {
    const { data: session, isPending, error } = authClient.useSession();
    const router = useRouter();
    const [loading, setLoading] = useState(true);

    // Simple client-side protection
    useEffect(() => {
        if (!isPending) {
            if (!session) {
                router.push("/signin");
            } else {
                setLoading(false);
            }
        }
    }, [session, isPending, router]);

    const handleSignOut = async () => {
        await authClient.signOut({
            fetchOptions: {
                onSuccess: () => {
                    router.push("/");
                },
            },
        });
    };

    if (isPending || loading) {
        return (
            <div className="min-h-screen bg-black flex items-center justify-center text-white">
                <div className="animate-pulse flex flex-col items-center gap-4">
                    <div className="h-12 w-12 rounded-full bg-zinc-800"></div>
                    <div className="h-4 w-32 rounded bg-zinc-800"></div>
                </div>
            </div>
        );
    }

    return (
        <div className="min-h-screen bg-gradient-to-br from-black via-zinc-950 to-slate-900 text-white font-sans">
            {/* Top nav to feel like a lab control bar */}
            <div className="border-b border-white/10 bg-black/70 backdrop-blur-md sticky top-0 z-40">
                <div className="mx-auto flex max-w-6xl items-center justify-between px-6 py-4">
                    <div className="flex items-center gap-2">
                        <Bot className="w-6 h-6 text-cyan-400" />
                        <span className="text-sm font-semibold tracking-tight">
                            Physical<span className="text-cyan-400">AI</span> Lab Console
                        </span>
                    </div>
                    <div className="flex items-center gap-4 text-xs text-zinc-400">
                        <span className="hidden md:inline-flex items-center gap-1">
                            <Activity className="w-3 h-3 text-emerald-400" />
                            Systems nominal
                        </span>
                        <div className="flex items-center gap-2">
                            <User className="w-4 h-4" />
                            <span className="text-xs">{session?.user?.name}</span>
                        </div>
                        <Button
                            variant="outline"
                            size="sm"
                            onClick={handleSignOut}
                            className="border-zinc-700 bg-zinc-900/60 hover:bg-zinc-900 text-zinc-200"
                        >
                            <LogOut className="w-3 h-3 mr-1" />
                            Sign Out
                        </Button>
                    </div>
                </div>
            </div>

            <main className="mx-auto flex max-w-6xl flex-col gap-8 px-6 py-8 md:py-10">
                {/* Hero row */}
                <section className="grid gap-6 md:grid-cols-[minmax(0,2fr)_minmax(0,1.2fr)] items-stretch">
                    <Card className="bg-zinc-900/80 border-zinc-800/80 shadow-[0_0_60px_rgba(0,0,0,0.7)]">
                        <CardHeader>
                            <CardTitle className="flex items-center justify-between">
                                <span className="text-2xl font-semibold tracking-tight">
                                    Welcome to the Humanoid Lab
                                </span>
                                <span className="rounded-full border border-emerald-400/40 bg-emerald-500/10 px-3 py-1 text-xs font-medium text-emerald-300">
                                    LIVE SESSION
                                </span>
                            </CardTitle>
                            <CardDescription className="text-zinc-400">
                                Orchestrate your learning across ROS 2, digital twins, NVIDIA Isaac, and VLA models from a single control surface.
                            </CardDescription>
                        </CardHeader>
                        <CardContent className="space-y-4">
                            <div className="grid gap-4 md:grid-cols-3">
                                <div className="rounded-lg border border-cyan-500/20 bg-cyan-500/5 p-3">
                                    <div className="mb-2 flex items-center gap-2 text-xs font-medium text-cyan-300">
                                        <Cpu className="w-4 h-4" />
                                        ROS 2 Nervous System
                                    </div>
                                    <p className="text-xs text-zinc-400">
                                        Understand nodes, topics, and services the way a real lab operator would.
                                    </p>
                                </div>
                                <div className="rounded-lg border border-purple-500/20 bg-purple-500/5 p-3">
                                    <div className="mb-2 flex items-center gap-2 text-xs font-medium text-purple-300">
                                        <Box className="w-4 h-4" />
                                        Digital Twin Arena
                                    </div>
                                    <p className="text-xs text-zinc-400">
                                        Move between Gazebo and Unity scenes to validate behaviors before touching hardware.
                                    </p>
                                </div>
                                <div className="rounded-lg border border-emerald-500/20 bg-emerald-500/5 p-3">
                                    <div className="mb-2 flex items-center gap-2 text-xs font-medium text-emerald-300">
                                        <Brain className="w-4 h-4" />
                                        AI Brain & VLA
                                    </div>
                                    <p className="text-xs text-zinc-400">
                                        Tie perception, language, and action together into full-stack Physical AI routines.
                                    </p>
                                </div>
                            </div>
                            <div className="flex flex-wrap items-center justify-between gap-3 text-xs text-zinc-400">
                                <div className="flex flex-wrap items-center gap-3">
                                    <span className="inline-flex items-center gap-1 rounded-full bg-zinc-900/70 px-3 py-1">
                                        <BookOpen className="w-3 h-3 text-cyan-400" />
                                        13-week industry curriculum
                                    </span>
                                    <span className="inline-flex items-center gap-1 rounded-full bg-zinc-900/70 px-3 py-1">
                                        <Bot className="w-3 h-3 text-orange-400" />
                                        RAG tutor connected to the textbook
                                    </span>
                                </div>
                                <Link
                                    href="/Physical-AI-Humanoid-Robotics-Textbook/docs/introduction/overview"
                                    target="_blank"
                                    className="inline-flex items-center gap-1 text-cyan-400 hover:text-cyan-300"
                                >
                                    <span className="text-xs">Open textbook</span>
                                    <LinkIcon className="w-3 h-3" />
                                </Link>
                            </div>
                        </CardContent>
                    </Card>

                    <Card className="bg-zinc-900/80 border-zinc-800/80">
                        <CardHeader>
                            <CardTitle className="text-sm font-semibold text-zinc-100">
                                Pilot Status
                            </CardTitle>
                            <CardDescription className="text-zinc-500">
                                Snapshot of your current learning run.
                            </CardDescription>
                        </CardHeader>
                        <CardContent className="space-y-4">
                            <div className="space-y-3 text-xs">
                                <div className="flex items-center justify-between">
                                    <span className="text-zinc-400">Email</span>
                                    <span className="font-medium text-zinc-100 truncate max-w-[55%] text-right">
                                        {session?.user?.email}
                                    </span>
                                </div>
                                <div className="flex items-center justify-between">
                                    <span className="text-zinc-400">Course Progress</span>
                                    <span className="font-medium text-cyan-400">0% • Boot sequence</span>
                                </div>
                                <div className="flex items-center justify-between">
                                    <span className="text-zinc-400">Hardware Profile</span>
                                    <span className="font-medium text-zinc-200">Simulation only</span>
                                </div>
                            </div>
                            <div className="space-y-2">
                                <p className="text-[11px] uppercase tracking-wide text-zinc-500">
                                    NEXT RECOMMENDED ACTION
                                </p>
                                <p className="text-xs text-zinc-300">
                                    Start with <span className="font-semibold text-cyan-300">ROS 2 Fundamentals</span>, then
                                    bring up your first digital twin in Gazebo before touching VLA agents.
                                </p>
                            </div>
                        </CardContent>
                    </Card>
                </section>

                {/* Modules + Lab schedule row */}
                <section className="grid gap-6 lg:grid-cols-[minmax(0,2fr)_minmax(0,1.2fr)] items-start">
                    <div className="space-y-4">
                        <h2 className="flex items-center gap-2 text-sm font-semibold text-zinc-300">
                            <BookOpen className="w-4 h-4 text-cyan-400" />
                            Continue your track
                        </h2>
                        <div className="grid gap-4 md:grid-cols-2">
                            {/* Module 1 */}
                            <Link href="/docs/introduction/overview" target="_blank" className="block">
                                <Card className="bg-zinc-900/80 border-zinc-800 hover:border-cyan-500/50 hover:shadow-[0_0_40px_rgba(34,211,238,0.25)] transition-all group h-full">
                                    <CardHeader>
                                        <CardTitle className="flex items-center justify-between text-sm font-semibold group-hover:text-cyan-300">
                                            Module 1: ROS 2 Basics
                                            <span className="rounded-full bg-zinc-900 px-2 py-0.5 text-[10px] text-zinc-400">
                                                Week 1–5
                                            </span>
                                        </CardTitle>
                                        <CardDescription>
                                            Foundations of the robotic nervous system: nodes, topics, and services.
                                        </CardDescription>
                                    </CardHeader>
                                    <CardContent>
                                        <div className="w-full bg-zinc-800 h-1.5 rounded-full overflow-hidden">
                                            <div className="bg-cyan-500 w-0 h-full"></div>
                                        </div>
                                        <p className="mt-2 text-[11px] text-zinc-500">0% complete • Start with ROS graph</p>
                                    </CardContent>
                                </Card>
                            </Link>

                            {/* Module 2 */}
                            <Link href="/docs/module-2-digital-twin/week-6-gazebo" target="_blank" className="block">
                                <Card className="bg-zinc-900/80 border-zinc-800 hover:border-purple-500/50 hover:shadow-[0_0_40px_rgba(168,85,247,0.25)] transition-all group h-full">
                                    <CardHeader>
                                        <CardTitle className="flex items-center justify-between text-sm font-semibold group-hover:text-purple-300">
                                            Module 2: Simulation
                                            <span className="rounded-full bg-zinc-900 px-2 py-0.5 text-[10px] text-zinc-400">
                                                Week 6–7
                                            </span>
                                        </CardTitle>
                                        <CardDescription>
                                            Gazebo and Unity environments for stress-testing behaviors safely.
                                        </CardDescription>
                                    </CardHeader>
                                    <CardContent>
                                        <div className="w-full bg-zinc-800 h-1.5 rounded-full overflow-hidden">
                                            <div className="bg-purple-500 w-0 h-full"></div>
                                        </div>
                                        <p className="mt-2 text-[11px] text-zinc-500">0% complete • Bring up your first world</p>
                                    </CardContent>
                                </Card>
                            </Link>

                            {/* Module 3 */}
                            <Link href="/docs/module-3-nvidia-isaac/week-8-intro" target="_blank" className="block">
                                <Card className="bg-zinc-900/80 border-zinc-800 hover:border-green-500/50 hover:shadow-[0_0_40px_rgba(34,197,94,0.25)] transition-all group h-full">
                                    <CardHeader>
                                        <CardTitle className="flex items-center justify-between text-sm font-semibold group-hover:text-green-300">
                                            Module 3: NVIDIA Isaac
                                            <span className="rounded-full bg-zinc-900 px-2 py-0.5 text-[10px] text-zinc-400">
                                                Week 8–10
                                            </span>
                                        </CardTitle>
                                        <CardDescription>
                                            Perception and reinforcement learning pipelines for embodied agents.
                                        </CardDescription>
                                    </CardHeader>
                                    <CardContent>
                                        <div className="w-full bg-zinc-800 h-1.5 rounded-full overflow-hidden">
                                            <div className="bg-green-500 w-0 h-full"></div>
                                        </div>
                                        <p className="mt-2 text-[11px] text-zinc-500">0% complete • Spin up Isaac Sim</p>
                                    </CardContent>
                                </Card>
                            </Link>

                            {/* Module 4 */}
                            <Link href="/docs/module-4-vla-humanoids/week-11-vla" target="_blank" className="block">
                                <Card className="bg-zinc-900/80 border-zinc-800 hover:border-orange-500/50 hover:shadow-[0_0_40px_rgba(249,115,22,0.3)] transition-all group h-full">
                                    <CardHeader>
                                        <CardTitle className="flex items-center justify-between text-sm font-semibold group-hover:text-orange-300">
                                            Module 4: VLA Models
                                            <span className="rounded-full bg-zinc-900 px-2 py-0.5 text-[10px] text-zinc-400">
                                                Week 11–13
                                            </span>
                                        </CardTitle>
                                        <CardDescription>
                                            Vision-Language-Action pipelines for humanoid decision-making.
                                        </CardDescription>
                                    </CardHeader>
                                    <CardContent>
                                        <div className="w-full bg-zinc-800 h-1.5 rounded-full overflow-hidden">
                                            <div className="bg-orange-500 w-0 h-full"></div>
                                        </div>
                                        <p className="mt-2 text-[11px] text-zinc-500">0% complete • Wire up your first VLA loop</p>
                                    </CardContent>
                                </Card>
                            </Link>
                        </div>
                    </div>

                    <Card className="bg-zinc-900/80 border-zinc-800">
                        <CardHeader>
                            <CardTitle className="flex items-center justify-between text-sm font-semibold text-zinc-100">
                                Lab Timeline
                                <span className="rounded-full bg-zinc-900 px-2 py-0.5 text-[10px] text-zinc-400">
                                    Today&apos;s plan
                                </span>
                            </CardTitle>
                            <CardDescription className="text-zinc-500">
                                A simple script of what to run in this session.
                            </CardDescription>
                        </CardHeader>
                        <CardContent className="space-y-3 text-xs">
                            <div className="flex items-start gap-3 rounded-md border border-zinc-800/80 bg-zinc-950/60 p-3">
                                <div className="mt-0.5 h-2 w-2 rounded-full bg-emerald-400" />
                                <div>
                                    <p className="font-medium text-zinc-100">Boot: ROS 2 Graph</p>
                                    <p className="text-zinc-500">
                                        Launch a minimal ROS 2 graph and inspect topics with <span className="font-mono text-[11px]">ros2 topic list</span>.
                                    </p>
                                </div>
                            </div>
                            <div className="flex items-start gap-3 rounded-md border border-zinc-800/80 bg-zinc-950/40 p-3">
                                <div className="mt-0.5 h-2 w-2 rounded-full bg-cyan-400" />
                                <div>
                                    <p className="font-medium text-zinc-100">Sim: Gazebo Scene</p>
                                    <p className="text-zinc-500">
                                        Load the week 6 Gazebo world and verify your humanoid stays stable for 60s.
                                    </p>
                                </div>
                            </div>
                            <div className="flex items-start gap-3 rounded-md border border-zinc-800/80 bg-zinc-950/20 p-3">
                                <div className="mt-0.5 h-2 w-2 rounded-full bg-purple-400" />
                                <div>
                                    <p className="font-medium text-zinc-100">AI Tutor</p>
                                    <p className="text-zinc-500">
                                        Ask the RAG tutor to explain any equation or diagram by selecting it in the textbook.
                                    </p>
                                </div>
                            </div>
                        </CardContent>
                    </Card>
                </section>
            </main>
        </div>
    );
}