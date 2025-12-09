import { HeroSection } from "@/components/landing/HeroSection";
import { ModuleCards } from "@/components/landing/ModuleCards";
import { FeaturesGrid } from "@/components/landing/FeaturesGrid";
import { ChatbotWidget } from "@/components/ChatbotWidget";

export default function Home() {
  return (
    <div className="min-h-screen">
      <HeroSection />
      <ModuleCards />
      <FeaturesGrid />

      {/* Footer */}
      <footer className="border-t border-white/10 py-12">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <div className="text-center text-muted">
            <p className="text-sm">
              © 2025 Physical AI & Humanoid Robotics. Built with Next.js and ❤️
            </p>
            <p className="text-xs mt-2">
              13-Week Interactive Textbook for Industry Practitioners
            </p>
          </div>
        </div>
      </footer>
      
      <ChatbotWidget />
    </div>
  );
}
