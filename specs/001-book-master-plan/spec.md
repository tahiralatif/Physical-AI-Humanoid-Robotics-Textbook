# Physical AI & Humanoid Robotics Textbook - Master Plan

## 1. Project Overview
**Goal**: Create a unified textbook to teach a course in **Physical AI & Humanoid Robotics**, bridging the gap between digital brains and physical bodies. This project is part of the "Hackathon I" by Panaversity to develop AI-native technical textbooks.

**Context**: The future of work involves partnership between humans, AI agents, and robots. This course empowers students to design, simulate, and deploy humanoid robots using cutting-edge tools like ROS 2, Gazebo, and NVIDIA Isaac.

---

## 2. Core Deliverables & Requirements
The project must include the following components:

### A. AI/Spec-Driven Book Creation (Base)
- **Platform**: Docusaurus 3 + GitHub Pages.
- **Tools**: Spec-Kit Plus & Claude Code.
- **Content**: Comprehensive textbook covering the defined course syllabus.

### B. Integrated RAG Chatbot (Base)
- **Functionality**: Embed a chatbot to answer user questions about the book content, including text-selection based queries.
- **Tech Stack**:
    - **LLM/SDK**: OpenAI Agents / ChatKit SDKs.
    - **Backend**: FastAPI.
    - **Database**: Neon Serverless Postgres.
    - **Vector DB**: Qdrant Cloud Free Tier.

### C. Bonus Features (Up to 200 Extra Points)
1.  **Reusable Intelligence (50 pts)**: Use Claude Code Subagents and Agent Skills for project creation.
    -   **Requirement**: Define and use subagents located in the `.claude/agents/` folder for specific tasks (e.g., `book-architect`, `frontend-dev`) whenever applicable to accelerate development.
2.  **User Authentication (50 pts)**:
    - Implement Signup/Signin using **Better-Auth**.
    - **Onboarding Survey**: Ask users about their software/hardware background at signup to enable personalization.
3.  **Content Personalization (50 pts)**:
    - Feature: "Personalize Content" button at the start of each chapter.
    - Logic: Adapts content based on the logged-in user's background (captured during signup).
4.  **Urdu Translation (50 pts)**:
    - Feature: "Translate to Urdu" button at the start of each chapter.

---

## 3. Course Curriculum: Physical AI & Humanoid Robotics

### Focus & Theme
**Embodied Intelligence**: AI systems that function in reality and comprehend physical laws.

### Module Breakdown
| Module | Title | Focus & Key Topics |
| :--- | :--- | :--- |
| **1** | **The Robotic Nervous System (ROS 2)** | Middleware for robot control. Nodes, Topics, Services, `rclpy`, URDF. |
| **2** | **The Digital Twin (Gazebo & Unity)** | Physics simulation. Gravity, collisions, Unity rendering, sensor simulation (LiDAR, Depth). |
| **3** | **The AI-Robot Brain (NVIDIA Isaac)** | Perception & training. Isaac Sim (Synthetic data), Isaac ROS (VSLAM, Nav2 for bipeds). |
| **4** | **Vision-Language-Action (VLA)** | LLMs + Robotics. OpenAI Whisper (Voice-to-Action), Cognitive Planning (LLM to ROS 2 actions). |

### Capstone Project: The Autonomous Humanoid
A simulated robot that:
1. Receives a voice command.
2. Plans a path & navigates obstacles.
3. Identifies an object (Computer Vision).
4. Manipulates the object.

---

## 4. Weekly Schedule
- **Weeks 1-2**: Intro to Physical AI, Embodied Intelligence, Sensors.
- **Weeks 3-5**: ROS 2 Fundamentals (Nodes, Topics, Launch files).
- **Weeks 6-7**: Robot Simulation (Gazebo URDF/SDF, Unity).
- **Weeks 8-10**: NVIDIA Isaac Platform (Isaac Sim, RL, Sim-to-Real).
- **Weeks 11-12**: Humanoid Robot Development (Kinematics, Bipedal Locomotion, Grasping).
- **Week 13**: Conversational Robotics (GPT Integration, Multi-modal interaction).

---

## 5. Timeline & Submission
- **Submission Deadline**: Sunday, Nov 30, 2025 at 06:00 PM.
- **Components to Submit**:
    1. Public GitHub Repo Link.
    2. Published Book Link (GitHub Pages/Vercel).
    3. Demo Video (< 90 seconds).
    4. WhatsApp Number.

---

## 6. Hardware Requirements (For Students)
Course is computationally heavy: **Physics Simulation + Visual Perception + Generative AI**.

### 1. "Digital Twin" Workstation (Critical)
Required for running NVIDIA Isaac Sim & Gazebo.
- **GPU**: NVIDIA RTX 4070 Ti (12GB VRAM) or higher (Ideal: RTX 3090/4090).
- **CPU**: Intel Core i7 (13th Gen+) or AMD Ryzen 9.
- **RAM**: 64 GB DDR5 (32 GB Min).
- **OS**: Ubuntu 22.04 LTS (Dual-boot mandatory).

### 2. "Physical AI" Edge Kit (The Brain)
To deploy ROS 2 nodes physically.
- **Brain**: NVIDIA Jetson Orin Nano (8GB) or Orin NX.
- **Vision**: Intel RealSense D435i or D455.
- **IMU**: BNO055 (or internal RealSense IMU).
- **Voice**: ReSpeaker USB Mic Array.

### 3. Robot Lab Options
- **Option A (Proxy)**: Unitree Go2 Edu (Quadruped) - Recommended budget option.
- **Option B (Miniature)**: Unitree G1 or Robotis OP3.
- **Option C (Premium / Sim-to-Real)**: Unitree G1 Humanoid.

---

## 7. Lab Architecture Summary
| Component | Hardware | Function |
| :--- | :--- | :--- |
| **Sim Rig** | PC (RTX 4080 + Ubuntu) | Isaac Sim, Gazebo, Unity, LLM Training |
| **Edge Brain** | Jetson Orin Nano | Inference Stack, Student Code Deployment |
| **Sensors** | RealSense + LiDAR | Real-world data feed |
| **Actuator** | Unitree Go2 / G1 | Motor execution |
