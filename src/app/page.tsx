import Link from "next/link";
import { Button } from "@/components/ui/button";
import { ArrowRight, Bot, Box, Brain, Cpu } from "lucide-react";

export default function Home() {
  return (
    <div className="min-h-screen bg-black text-white selection:bg-cyan-500/30 font-sans">
      {/* Navbar Placeholder */}
      <header className="fixed top-0 left-0 right-0 border-b border-white/10 bg-black/50 backdrop-blur-md z-50">
        <div className="container mx-auto px-6 h-16 flex items-center justify-between">
          <div className="flex items-center gap-2 font-bold text-xl tracking-tighter cursor-pointer">
            <Bot className="text-cyan-400" />
            <span>Physical<span className="text-cyan-400">AI</span></span>
          </div>
          <div className="hidden md:flex items-center gap-6 text-sm font-medium text-zinc-400">
            <Link href="#modules" className="hover:text-cyan-400 transition-colors">Curriculum</Link>
            <Link href="#features" className="hover:text-cyan-400 transition-colors">Platform</Link>
            <Link href="https://github.com/tahiralatif/Physical-AI-Humanoid-Robotics-Textbook" className="hover:text-cyan-400 transition-colors">GitHub</Link>
          </div>
          <div className="flex items-center gap-4">
            <Link
              href="/signin"
              className="text-sm font-medium text-zinc-400 hover:text-white transition-colors"
            >
              Sign In
            </Link>
            <Link href="/signup">
              <Button
                variant="outline"
                className="border-cyan-500/50 text-cyan-400 hover:bg-cyan-950 hover:text-cyan-300 hover:shadow-[0_0_30px_rgba(34,211,238,0.35)] transition-shadow"
              >
                Get Started
              </Button>
            </Link>
          </div>
        </div>
      </header>

      {/* Hero */}
      <section className="pt-32 pb-20 md:pt-48 md:pb-32 container mx-auto px-6 relative overflow-hidden">
        <div className="absolute top-0 left-1/2 -translate-x-1/2 w-[1000px] h-[500px] bg-cyan-500/20 blur-[120px] rounded-full pointer-events-none" />

        <div className="max-w-4xl mx-auto text-center relative z-10">
          <div className="inline-flex items-center gap-2 px-3 py-1 rounded-full bg-cyan-950/30 border border-cyan-500/20 text-cyan-400 text-xs font-medium mb-8">
            <span className="relative flex h-2 w-2">
              <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-cyan-400 opacity-75"></span>
              <span className="relative inline-flex rounded-full h-2 w-2 bg-cyan-500"></span>
            </span>
            Hackathon I: Advanced Robotics
          </div>
          <h1 className="text-5xl md:text-7xl font-bold tracking-tight mb-8 bg-gradient-to-b from-white to-zinc-500 bg-clip-text text-transparent">
            Bridge the Gap Between <br /> Digital Brains & Physical Bodies
          </h1>
          <p className="text-lg md:text-xl text-zinc-400 mb-10 max-w-2xl mx-auto leading-relaxed">
            A unified textbook to teach Physical AI & Humanoid Robotics.
            Master ROS 2, Gazebo, NVIDIA Isaac, and VLA models in a single platform.
          </p>
          <div className="flex flex-col sm:flex-row items-center justify-center gap-4">
            <Link href="/signin">
              <Button
                size="lg"
                className="bg-cyan-500 hover:bg-cyan-400 text-black font-semibold h-12 px-8 hover:shadow-[0_0_45px_rgba(34,211,238,0.55)] transition-shadow"
              >
                Get your Panaverse ID <ArrowRight className="ml-2 w-4 h-4" />
              </Button>
            </Link>
            <Link href="/Physical-AI-Humanoid-Robotics-Textbook/docs/introduction/overview">
              <Button
                size="lg"
                variant="outline"
                className="border-white/10 hover:bg-white/5 h-12 px-8"
              >
                View Curriculum
              </Button>
            </Link>
          </div>
        </div>
      </section>

      {/* Features Grid */}
      <section id="modules" className="py-20 bg-zinc-900/50">
        <div className="container mx-auto px-6">
          <div className="text-center mb-16">
            <h2 className="text-3xl font-bold tracking-tight mb-4">Comprehensive Curriculum</h2>
            <p className="text-zinc-400 max-w-xl mx-auto">From bare-metal ROS 2 control to advanced multimodal LLM integration.</p>
          </div>
          <div className="grid md:grid-cols-2 lg:grid-cols-4 gap-6">
            {/* Module 1 */}
            <div className="p-6 rounded-2xl bg-black border border-white/10 hover:border-cyan-500/50 transition-colors group cursor-default">
              <div className="w-12 h-12 rounded-lg bg-zinc-900 flex items-center justify-center mb-4 group-hover:bg-cyan-950 transition-colors">
                <Cpu className="w-6 h-6 text-zinc-400 group-hover:text-cyan-400" />
              </div>
              <h3 className="text-lg font-bold mb-2">The Nervous System</h3>
              <p className="text-zinc-500 text-sm">Master ROS 2 nodes, topics, and services to control hardware.</p>
            </div>
            {/* Module 2 */}
            <div className="p-6 rounded-2xl bg-black border border-white/10 hover:border-purple-500/50 transition-colors group cursor-default">
              <div className="w-12 h-12 rounded-lg bg-zinc-900 flex items-center justify-center mb-4 group-hover:bg-purple-950 transition-colors">
                <Box className="w-6 h-6 text-zinc-400 group-hover:text-purple-400" />
              </div>
              <h3 className="text-lg font-bold mb-2">The Digital Twin</h3>
              <p className="text-zinc-500 text-sm">Simulate physics and environments in Gazebo & Unity.</p>
            </div>
            {/* Module 3 */}
            <div className="p-6 rounded-2xl bg-black border border-white/10 hover:border-green-500/50 transition-colors group cursor-default">
              <div className="w-12 h-12 rounded-lg bg-zinc-900 flex items-center justify-center mb-4 group-hover:bg-green-950 transition-colors">
                <Brain className="w-6 h-6 text-zinc-400 group-hover:text-green-400" />
              </div>
              <h3 className="text-lg font-bold mb-2">The AI Brain</h3>
              <p className="text-zinc-500 text-sm">Train perceptions models using NVIDIA Isaac Sim & ROS.</p>
            </div>
            {/* Module 4 */}
            <div className="p-6 rounded-2xl bg-black border border-white/10 hover:border-orange-500/50 transition-colors group cursor-default">
              <div className="w-12 h-12 rounded-lg bg-zinc-900 flex items-center justify-center mb-4 group-hover:bg-orange-950 transition-colors">
                <Bot className="w-6 h-6 text-zinc-400 group-hover:text-orange-400" />
              </div>
              <h3 className="text-lg font-bold mb-2">Vision-Language-Action</h3>
              <p className="text-zinc-500 text-sm">Integrate LLMs with Robotics for cognitive planning.</p>
            </div>
          </div>
        </div>
      </section>

      {/* Footer */}
      <footer className="py-12 border-t border-white/10 bg-black text-center text-zinc-600 text-sm">
        <p className="mb-2">&copy; 2025 Physical AI Textbook. All rights reserved.</p>
        <p>Built for the Physical AI Hackathon.</p>
      </footer>
    </div>
  );
}
