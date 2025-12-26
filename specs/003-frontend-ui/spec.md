# Specification: Frontend UI (Next.js)

**Domain**: 003-frontend-ui
**Focus**: User Interface, Authentication, Dashboard, Content Consumption Wrapper.

## 1. Goal
Create a modern, "Premium" Next.js application that serves as the primary entry point for users. It will handle user authentication (Better-Auth), personalized dashboards, and wrap the Docusaurus content (or link to it seamlessly). It also houses the RAG Chatbot Widget.

## 2. Requirements

### Authentication
*   **Library**: Better-Auth.
*   **Flow**: Signup -> Survey (Background Info) -> Dashboard.
*   **Survey Fields**: Software Skills (Python/C++), Hardware Access (Sim/Jetson), Learning Goal.

### Integration
*   **Docusaurus Connection**: The Next.js app should link to the Docusaurus site. Ideally, Docusaurus runs on a subdomain or subpath (`/docs`), while Next.js runs the root (`/`) and app dashboard (`/app`).
*   **RAG Chatbot**: The Chat Widget must be persistent across the UI.

### Aesthetics
*   **Design System**: TailwindCSS + Shadcn/UI.
*   **Theme**: Dark Mode default, Cyberpunk/Robotics aesthetic (Clean, Neon accents).
*   **Responsive**: Mobile-first design.

## 3. Subagent Requirement
*   Use `frontend-architect` agent from `.claude/agents/` to scaffolding components and pages.
