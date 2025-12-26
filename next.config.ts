import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  async rewrites() {
    return [
      {
        source: "/docs",
        destination: "http://localhost:3001/Physical-AI-Humanoid-Robotics-Textbook/docs",
      },
      {
        source: "/docs/:path*",
        destination: "http://localhost:3001/Physical-AI-Humanoid-Robotics-Textbook/docs/:path*",
      },
      {
        source: "/Physical-AI-Humanoid-Robotics-Textbook/:path*",
        destination: "http://localhost:3001/Physical-AI-Humanoid-Robotics-Textbook/:path*",
      },
    ];
  },
};

export default nextConfig;