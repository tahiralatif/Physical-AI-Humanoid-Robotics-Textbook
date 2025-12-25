# Plan: Frontend UI (Next.js)

**Domain**: 003-frontend-ui
**Spec**: specs/003-frontend-ui/spec.md

## 1. Directory Structure
```
nextjs/
├── app/
│   ├── (auth)/             # Signup/Signin routes
│   ├── dashboard/          # User Dashboard
│   └── page.tsx            # Landing Page
├── components/
│   ├── ui/                 # Shadcn Atoms
│   └── widget/             # Chatbot Widget
├── lib/
│   └── auth.ts             # Better-Auth Config
```

## 2. Implementation Steps
1.  **Initialize**: `npx create-next-app@latest nextjs` (TS, Tailwind, App Router).
2.  **Dependencies**: Install `better-auth`, `lucide-react`, `framer-motion`.
3.  **Auth Setup**: Configure Better-Auth with Neon Postgres adapter.
4.  **UI Construction**:
    *   Landing Page: Hero section, Features.
    *   Auth Pages: Custom forms with "Survey" steps.
    *   Dashboard: Progress tracking, Link to Book Chapters.
5.  **Chat Integration**: Build the Chat Bubble widget that talks to FastAPI.

## 3. Subagent
*   Execute `frontend-architect` to build the Layout and Theme content.
