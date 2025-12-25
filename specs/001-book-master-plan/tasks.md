# Tasks: Physical AI & Humanoid Robotics Textbook

**Feature**: 001-book-master-plan
**Date**: 2025-12-18
**Spec**: specs/001-book-master-plan/spec.md
**Plan**: specs/001-book-master-plan/plan.md

---

## Phase 1: Foundation Setup

*   [x] T001 Initialize Docusaurus 3 Project with Spec-Kit Plus in `docusaurus/`
*   [x] T002 Initialize FastAPI Project (Python 3.10+) in `fastapi/`
*   [x] T003 Set up `uv` package manager and `requirements.txt`
*   [x] T004 Provision Neon Serverless Postgres DB and obtain connection string
*   [x] T005 Create SQL Tables in Neon: `users`, `profiles`, `chat_history`, `usage_logs`
*   [x] T006 Provision Qdrant Cloud Cluster and get API Key
*   [x] T007 Configure Environment Variables (`.env`) for both projects
*   [x] T008 Configure GitHub Actions for Docusaurus Deployment (Pages)
*   [x] T009 Configure GitIgnore and Repo Root structure
*   [x] T009b Define core Subagents in `.claude/agents/` (e.g., `project-manager.json`, `tech-lead.json`) to assist in Phase 2+

## Phase 2: The Core Textbook (Docusaurus)

*   [x] T010 Configure Docusaurus `docusaurus.config.js` (Title, URL, Presets)
*   [x] T011 Implement Sidebar Structure in `sidebars.js` (4 Modules, 13 Weeks)
*   [x] T012 Design Landing Page `src/pages/index.js` with Feature Cards
*   [x] T013 Create **Module 1** (ROS 2) Content Files (Weeks 3-5)
    *   [x] T013a Setup `docs/module-1-ros2/intro.md`
*   [x] T014 Create **Module 2** (Digital Twin) Content Files (Weeks 6-7)
*   [x] T015 Create **Module 3** (NVIDIA Isaac) Content Files (Weeks 8-10)
*   [x] T016 Create **Module 4** (VLA) Content Files (Weeks 11-13)
*   [x] T017 Write Capstone Project Guide `docs/capstone/autonomous-humanoid.md`
*   [x] T018 Write Hardware Setup Guide (Workstation + Jetson + Robot)
*   [x] T019 Implement Glossary and References section

## Phase 3: The Intelligence (RAG Chatbot)

*   [x] T020 Implement Data Ingestion Script `scripts/ingest.py`
    *   [x] T020a Recursive Markdown Parser to extraction sections
    *   [x] T020b Chunking Logic (overlap, max tokens)
    *   [x] T020c OpenAI Embedding Generation (`text-embedding-3-small`)
    *   [x] T020d Qdrant Upsert Logic
*   [x] T021 Create FastAPI `POST /api/chat` Endpoint
*   [x] T022 Implement Vector Search Service (Retrieval)
    *   [x] T022a Qdrant Client Integration
    *   [x] T022b Hybrid Search (Keyword + Semantic) logic
*   [x] T023 Implement LLM Service (Generation)
    *   [x] T023a Construct System Prompt with Context
    *   [x] T023b Integrate OpenAI Chat Completion (`gpt-4o-mini`)
*   [x] T024 Create React Chat Widget `src/components/ChatWidget.js`
*   [x] T025 Integrate Chat Widget into Docusaurus Layout (Swizzle `Layout`)
*   [x] T026 Implement "Ask about Selection" feature (Context Menu or Button)

## Phase 4: User Authentication (Bonus)

*   [x] T027 Implement Better-Auth (or similar) in FastAPI `api/auth.py`
*   [x] T028 Create Signup Page in Docusaurus (Custom React Page)
*   [x] T029 Implement Onboarding Survey Form (Expertise, Hardware, Goals)
*   [x] T030 Create Profile Management Endpoint `POST /api/user/profile`
*   [x] T031 Store User Metadata in Neon `profiles` table

## Phase 5: Content Personalization (Bonus)

*   [x] T032 Implement "Personalize" Button Component in `src/theme/DocItem`
*   [x] T033 Create Personalization Endpoint `POST /api/personalize/{chapter_id}`
*   [x] T034 Implement LLM Logic: "Rewrite X for [Profile]" in FastAPI
*   [x] T035 Implement Caching Strategy (Neon `personalized_content` table)
*   [x] T036 Frontend: Dynamically render personalized HTML content

## Phase 6: Urdu Translation (Bonus)

*   [x] T037 Implement "Translate to Urdu" Toggle in Navbar/DocItem
*   [x] T038 Create Translation Endpoint `POST /api/translate/{chapter_id}`
*   [x] T039 Implement LLM Translation Logic (Preserving Code/Technical Terms)
*   [x] T040 Frontend: Apply RTL CSS class when Urdu is active

## Phase 7: Reusable Intelligence (Subagents)

*   [x] T041 Define & Implement Claude Code Subagent: `book-architect` in `.claude/agents/book-architect.json`
*   [x] T042 Define & Implement Claude Code Subagent: `quiz-generator` in `.claude/agents/quiz-generator.json`
*   [x] T043 Use `book-architect` to review Module content structure
*   [x] T043b Document usage of `.claude` agents in `docs/agent_usage.md`
*   [ ] T045 Deploy to Production (GitHub Pages + Vercel)
