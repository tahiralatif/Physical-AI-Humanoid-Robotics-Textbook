---
id: 003
title: configure-nextjs-rewrites-for-docusaurus
stage: implement
date: 2025-12-18
surface: agent
model: gemini-2.0-flash-exp
feature: 001-book-master-plan
branch: main
user: Matz
command: implement
labels: ["nextjs", "docusaurus", "proxy", "rewrites"]
links:
  spec: specs/001-book-master-plan/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - nextjs/next.config.ts
tests:
 - null
---

## Prompt

yes

## Response snapshot

Configured `next.config.ts` to proxy all `/docs` requests to the Docusaurus server (port 3001). This unifies the Next.js marketing/auth site with the Docusaurus textbook content under a single domain during development.

## Outcome

- ✅ Impact: Unified routing between Next.js and Docusaurus.
- 🧪 Tests: Requires both servers running simultaneously (Next.js on 3000, Docusaurus on 3001).
- 📁 Files: Modified `nextjs/next.config.ts`.
- 🔁 Next prompts: "Implement Docusaurus Personalize button", "Integrate Chat Widget".
- 🧠 Reflection: This is a standard pattern for hybrid SSG/SPA applications.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
