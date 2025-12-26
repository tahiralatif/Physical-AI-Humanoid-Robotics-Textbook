# Plan: Physical AI & Humanoid Robotics Textbook Project

**Feature**: 001-book-master-plan
**Date**: 2025-12-18
**Spec**: specs/001-book-master-plan/spec.md
**Tasks**: specs/001-book-master-plan/tasks.md

---

## 1. Architectural Overview

The system is designed as a **Hybrid AI-Native Application**, combining a static documentation site with dynamic AI services.

### Components
1.  **Frontend (The Book)**:
    -   **Tech**: Docusaurus 3 (React-based SSG).
    -   **Host**: GitHub Pages.
    -   **Role**: Displays textbook content, integrates Chatbot Widget and Personalization Controls.
2.  **Backend (The Intelligence)**:
    -   **Tech**: FastAPI (Python).
    -   **Role**: Handles RAG pipeline, Auth, Personalization logic, and Translation services.
    -   **Host**: Vercel (or similar for Python serverless) / Render.
3.  **Database Layer**:
    -   **Relational**: Neon Serverless Postgres (Users, Profiles, Logs).
    -   **Vector**: Qdrant Cloud (Embeddings for RAG).
4.  **AI Services**:
    -   **LLM**: OpenAI (GPT-4o / GPT-4o-mini).
    -   **Agents**: ChatKit SDKs / OpenAI Agents API.
    -   **Development Subagents**: Custom Claude Code agents defined in `.claude/agents/` to handle specific domains (Documentation, Frontend, Backend).

---

## 2. Technical Stack & Key Decisions

### A. Book Engine: Docusaurus
-   **Why**: Optimized for documentation, supports MDX (React in Markdown), easy to extend with Spec-Kit Plus.
-   **Customization**: Custom React components for "Personalize" and "Translate" buttons injected via Swizzling or MDX components.

### B. Chatbot Pipeline (RAG)
-   **Ingestion**: Python script parses `.md` files -> chunks -> OpenAI Embeddings -> Qdrant.
-   **Retrieval**: `qdrant-client` searches for relevant chunks based on user query + text selection context.
-   **Generation**: OpenAI Chat Completion API generates answer with citations.

### C. Authentication & Personalization
-   **Auth**: **Better-Auth** (seamless integration with Next.js/FastAPI).
-   **Profile**: Stored in Neon. Fields: `expertise_level` (Beginner/Expert), `hardware` (Sim-only/Jetson/Robot), `goals`.
-   **Personalization Logic**:
    -   Fetch original chapter markdown.
    -   Prompt LLM: "Rewrite this section for a [Beginner] who has [No Robot]..."
    -   Cache result in Postgres `personalized_content` table to reduce latency/cost.

---

## 3. Data Flow

### Chatbot Query
1.  User types query in Docusaurus Widget.
2.  Frontend sends JSON `{query, selected_text, session_id}` to FastAPI.
3.  FastAPI converts `query` to vector.
4.  Qdrant returns Top-K chunks.
5.  FastAPI constructs prompt with Context + History.
6.  LLM generates response.
7.  Response sent back to Frontend.

### Content Personalization
1.  User Clicks "Personalize".
2.  Frontend checks JWT status.
3.  Request sent to `POST /api/personalize/{chapter_id}`.
4.  Backend checks Cache.
    -   *Hit*: Return cached HTML/Markdown.
    -   *Miss*: Call LLM -> Store in DB -> Return.
5.  Frontend renders new content dynamically.

---

## 4. Implementation Strategy

### Step 1: Foundation (Setup)
-   Initialize Repo.
-   Setup Docusaurus skeleton.
-   Setup FastAPI skeleton.
-   Provision Databases.

### Step 2: Content (The Core)
-   Write Spec-Kit Plus driven content for all 4 Modules.
-   Ensure structured Frontmatter for metadata.

### Step 3: Intelligence (Chatbot)
-   Build Ingestion Script.
-   Build Chat API.
-   Create Chat Widget component.

### Step 4: Engagement (Bonus)
-   Implement Auth & Survey.
-   Implement Personalization & Translation APIs.
-   Integrate "Bonus" UI toggles in Docusaurus.

### Step 5: Agentic Acceleration
-   Define subagents in `.claude/agents/` (e.g., `rag-architect`, `docusaurus-expert`).
-   Utilize these agents for repetitive or specialized tasks throughout the development lifecycle.

---

## 5. Verification Plan
-   **Build Check**: `npm run build` passes for Docusaurus.
-   **Deployment**: Site accessible on GitHub Pages.
-   **RAG Accuracy**: Query "What is ROS 2?" returns accurate definition from Module 1.
-   **Bonus**: Login, Personalize Chapter 1, Switch to Urdu -> Verify changes.
