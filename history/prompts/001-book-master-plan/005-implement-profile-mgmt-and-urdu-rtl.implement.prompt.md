---
id: 005
title: implement-profile-mgmt-and-urdu-rtl
stage: implement
date: 2025-12-18
surface: agent
model: gemini-2.0-flash-exp
feature: 001-book-master-plan
branch: main
user: Matz
command: implement
labels: ["fastapi", "neon", "docusaurus", "rtl", "urdu", "gemini"]
links:
  spec: specs/001-book-master-plan/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - .claude/agents/project-manager.md
 - .claude/agents/tech-lead.md
 - fastapi/app/models/user.py
 - fastapi/app/core/db.py
 - fastapi/app/services/user_service.py
 - fastapi/app/api/endpoints/user.py
 - fastapi/app/api/api.py
 - docusaurus/src/components/TranslateToggle.tsx
 - docusaurus/src/components/TranslateToggle.module.css
 - docusaurus/src/theme/DocItem/Content/index.tsx
 - docusaurus/src/components/PersonalizeButton.tsx
 - fastapi/app/services/openai_service.py
 - fastapi/app/core/config.py
 - specs/001-book-master-plan/tasks.md
tests:
 - null
---

## Prompt

openAI key for chatbot? isted openai key use gemini api key and use OpenAIChatCompletions model

## Response snapshot

Implemented comprehensive platform updates:
1.  **AI Integration**: Re-routed `openai_service` to use **Gemini API** via the OpenAI-compatible endpoint. Added support for Gemini-specific models (`gemini-1.5-flash`, `text-embedding-004`).
2.  **Profile Management**: Added FastAPI models, services, and endpoints for User Profiles stored in **Neon Postgres**.
3.  **Urdu & Personalization**:
    *   Created `TranslateToggle` component in Docusaurus.
    *   Integrated `react-markdown` to render AI-generated content dynamically.
    *   Applied **RTL (Right-to-Left)** styling for Urdu translations.
    *   Connected `PersonalizeButton` to the backend service.
4.  **Subagents**: Added `project-manager` and `tech-lead` agents as per Phase 7 requirements.

## Outcome

- ✅ Impact: Platform is now "AI-Native" with Gemini support, profile persistence, and multi-language/personalized rendering.
- 🧪 Tests: Ingestion script updated for Gemini vector size (768). Requires testing with real keys.
- 📁 Files: 14 files created/modified.
- 🔁 Next prompts: "Provide Qdrant and Gemini keys", "Run ingestion", "Verify RTL layout".
- 🧠 Reflection: Using Gemini's OpenAI compatibility made the switch seamless. The `DocItem/Content` wrapper is now the central hub for AI content injection.

## Evaluation notes (flywheel)

- Failure modes observed: `replace` tool failed due to `...` in search string. Fixed by being precise.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
