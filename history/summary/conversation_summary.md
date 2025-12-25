# Project Context Summary: Physical AI & Humanoid Robotics Textbook

**Date**: 2025-12-18
**Status**: Development Complete / Ready for Deployment

## 🎯 Overview
This project is a high-standard, AI-native technical textbook platform developed for the Panaversity Hackathon I. It bridges the gap between Next.js (Marketing/Auth), Docusaurus (Educational Content), and a FastAPI AI Brain.

## 🛠️ Technical Stack
- **Frontend**: Next.js 15 (Tailwind CSS, Shadcn/UI, Lucide Icons).
- **Book Engine**: Docusaurus 3 (MDX, Custom Swizzled Components).
- **Backend**: FastAPI (Python 3.10+).
- **AI Brain**: Google Gemini API (via OpenAI-compatible endpoint).
- **Databases**:
    - **Relational**: Neon Serverless Postgres (User Profiles, Content Caching).
    - **Vector**: Qdrant Cloud (RAG Knowledge Base).

## ✨ Key Features Implemented

### 1. Unified Platform Integration
- **Next.js Rewrites**: Configured `next.config.ts` to proxy `/docs` to the Docusaurus server, creating a seamless single-domain experience.
- **Premium UI**: Dark-mode, high-fidelity landing page and student dashboard.

### 2. AI Humanoid Tutor (RAG Chatbot)
- **Global Widget**: Floating chat bubble available on every textbook page.
- **Ask about Selection**: Intelligent UI feature where selecting text allows the student to ask the AI specifically about that context.
- **Ingestion Script**: `fastapi/scripts/ingest.py` parses Markdown, generates embeddings (Gemini), and upserts to Qdrant.

### 3. Personalization & Localization
- **One-Click Personalize**: Dynamically rewrites chapters based on the user's expertise and hardware (Simulation vs. Real Robot).
- **Neon Caching**: Personalized content is cached in Postgres for instant loading.
- **Urdu Translation**: Full support for Urdu translation with **RTL (Right-to-Left)** typography and layout logic in CSS.

### 4. Agentic Development
- Defined and implemented custom Claude Subagents:
    - `book-architect`: Curriculum & pedagogical review.
    - `quiz-generator`: Assessment & Rubric creation.
    - `tech-lead`: Code quality & Security auditing.
    - `project-manager`: Task tracking and health reports.

## 🚀 How to Resume Development

### 1. Environment Setup
Ensure `.env` files are configured in both `fastapi/` and `nextjs/`.
- **FastAPI Keys**: `GEMINI_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`, `DATABASE_URL`.
- **Next.js Keys**: `DATABASE_URL`.

### 2. Running the Stack
- **Backend**: `cd fastapi && uvicorn app.main:app --reload`
- **Book**: `cd docusaurus && npm start -- --port 3001`
- **Frontend**: `cd nextjs && npm run dev`

### 3. Data Ingestion
If you update the textbook content in `docusaurus/docs/`, run:
`cd fastapi && python scripts/ingest.py`

## 🏁 Current Task Status
All technical milestones in `specs/001-book-master-plan/tasks.md` are marked **[x]**. The project is ready for Demo Video recording and Production Deployment.
