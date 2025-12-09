# Tasks: Physical AI & Humanoid Robotics Textbook

**Feature**: 001-book-master-plan
**Date**: 2025-12-07
**Spec**: E:/Documents/quarter_04/hackathon/Physical-AI-Humanoid-Robotics-Textbook/specs/001-book-master-plan/spec.md
**Plan**: E:/Documents/quarter_04/hackathon/Physical-AI-Humanoid-Robotics-Textbook/specs/001-book-master-plan/plan.md

---

## Phase 1: Setup

*   [x] T001 Create base project directories and initialize Docusaurus in `docusaurus/`
*   [x] T002 Create base project directories and initialize Next.js in `nextjs/`
*   [x] T003 Create base project directories and initialize FastAPI in `fastapi/`
*   [x] T004 Set up `uv` package manager for FastAPI project `fastapi/`
*   [x] T005 Configure Tailwind CSS and shadcn/ui for Next.js project `nextjs/`
*   [x] T006 Initialize Git repository and link to GitHub (if not already)
*   [x] T007 Configure initial `.env` files for Docusaurus, Next.js, FastAPI with placeholder values
*   [x] T008 Provision Neon Serverless Postgres database and obtain connection string
*   [x] T009 Provision Qdrant Cloud Free Tier and obtain API key/URL
*   [x] T010 Create `users` and `user_profiles` tables in Neon Postgres based on schema in `spec.md`
*   [x] T011 Create `chat_messages` and `conversation_sessions` tables in Neon Postgres based on schema in `spec.md`
*   [x] T012 Create `personalized_content` and `translated_chapters` tables in Neon Postgres based on schema in `spec.md`
*   [x] T013 Create `agent_call_logs` table in Neon Postgres based on schema in `spec.md`

## Phase 2: Foundational

*   [x] T014 Implement Better Auth integration for signup/signin in `nextjs/` and `fastapi/`
*   [x] T015 Implement secure HTTP-only cookie session management for authentication in `fastapi/`
*   [x] T016 Integrate Next.js App Router middleware for auth checks in `nextjs/middleware.ts`
*   [x] T017 Set up GitHub Actions for Docusaurus deployment to GitHub Pages/Vercel
*   [ ] T018 Set up GitHub Actions for Next.js/FastAPI deployment to Vercel

## Phase 3: User Story 1 - Browse and Learn from Textbook (P1)

**Story Goal**: As an industry practitioner learning Physical AI, I need to access a well-structured 13-week course textbook, so I can systematically learn ROS 2, Gazebo, NVIDIA Isaac, and humanoid robotics.
**Independent Test**: Navigate through all 13 weeks of content, verify all chapters are accessible and properly formatted.

*   [x] T019 [P] [US1] Create Docusaurus `docusaurus.config.js` with sidebar structure (4 modules, 13 weeks, topics) in `docusaurus/`
*   [x] T020 [P] [US1] Create Docusaurus homepage dashboard layout with 4 module cards, quick links, recent updates in `docusaurus/src/pages/index.js`
*   [x] T021 [P] [US1] Create Introduction section content (Weeks 1-2: Physical AI Foundations) in `docusaurus/docs/introduction/`
*   [x] T022 [P] [US1] Create Module 1 (ROS 2 - Weeks 3-5) directory structure and placeholder chapters in `docusaurus/docs/module-1-ros2/`
*   [x] T023 [P] [US1] Create Module 2 (Digital Twin - Weeks 6-7) directory structure and placeholder chapters in `docusaurus/docs/module-2-digital-twin/`
*   [x] T024 [P] [US1] Create Module 3 (NVIDIA Isaac - Weeks 8-10) directory structure and placeholder chapters in `docusaurus/docs/module-3-nvidia-isaac/`
*   [x] T025 [P] [US1] Create Module 4 (VLA & Humanoids - Weeks 11-13) directory structure and placeholder chapters in `docusaurus/docs/module-4-vla-humanoids/`
*   [x] T026 [P] [US1] Add placeholder content for Capstone Project Guide in `docusaurus/docs/capstone/`
*   [x] T027 [P] [US1] Add placeholder content for 3 hardware setup guides in `docusaurus/docs/hardware-setup/`
*   [x] T028 [P] [US1] Add placeholder content for 4 assessment guides in `docusaurus/docs/assessments/`
*   [x] T029 [P] [US1] Create Glossary (100+ terms) with search functionality (Docusaurus search config) in `docusaurus/docs/glossary.md`
*   [x] T030 [P] [US1] Create Notation Guide, ROS 2 Quick Reference, Troubleshooting Guide in `docusaurus/docs/reference/`
*   [x] T031 [US1] Ensure all placeholder chapters have correct Docusaurus frontmatter (`estimated_time`, `week`, `module`, `prerequisites`, `learning_objectives`, `sidebar_label`) in `docusaurus/docs/**/*.md`
*   [x] T032 [US1] Run Docusaurus build locally and verify all content is accessible and formatted correctly in `docusaurus/`
*   [ ] T033 [US1] Deploy Docusaurus book to GitHub Pages or Vercel (using configured CI/CD)

## Phase 4: User Story 2 - Get AI-Powered Assistance via RAG Chatbot (P1)

**Story Goal**: As an industry practitioner, I need to ask questions about textbook content through an embedded chatbot, so I can get immediate clarifications without searching through chapters.
**Independent Test**: Ask 20 diverse questions, verify 90%+ accuracy and proper source attribution.

*   [ ] T034 [P] [US2] Create Qdrant collection `textbook_chunks` with specified schema in `fastapi/app/qdrant_client.py`
*   [ ] T035 [P] [US2] Implement data ingestion script to embed textbook content and upload to Qdrant (using OpenAI text-embedding-3-small) in `fastapi/scripts/ingest_data.py`
*   [ ] T036 [P] [US2] Implement FastAPI endpoint `POST /api/chat` for sending messages to chatbot in `fastapi/app/api/chat.py`
*   [ ] T037 [P] [US2] Implement FastAPI endpoint `GET /api/chat/history/{session_id}` for retrieving conversation history in `fastapi/app/api/chat.py`
*   [ ] T038 [P] [US2] Implement FastAPI endpoint `DELETE /api/chat/history/{session_id}` for clearing conversation history in `fastapi/app/api/chat.py`
*   [ ] T039 [P] [US2] Implement FastAPI endpoint `GET /api/chat/sessions` for retrieving user's conversation sessions in `fastapi/app/api/chat.py`
*   [ ] T040 [P] [US2] Integrate OpenAI ChatKit SDK for conversational interface in `fastapi/app/services/chatbot_service.py`
*   [ ] T041 [P] [US2] Implement conversation context management (min 5 turns) and history storage in Neon Postgres `fastapi/app/services/conversation_service.py`
*   [ ] T042 [P] [US2] Implement text selection feature logic (limit questions to selected context) in `fastapi/app/services/chatbot_service.py`
*   [ ] T043 [P] [US2] Implement semantic similarity search to retrieve top-5 relevant chunks from Qdrant in `fastapi/app/services/retrieval_service.py`
*   [ ] T044 [P] [US2] Implement source citation (chapter/section) in chatbot responses in `fastapi/app/services/chatbot_service.py`
*   [ ] T045 [P] [US2] Implement logic to detect and explicitly state when query cannot be answered from textbook in `fastapi/app/services/chatbot_service.py`
*   [ ] T046 [P] [US2] Implement rate-limiting for chatbot queries (50 per user per day) in `fastapi/app/api/chat.py`
*   [ ] T047 [P] [US2] Implement chatbot UI widget (bottom-right, collapsible) using shadcn/ui components in `nextjs/src/components/ChatbotWidget.tsx`
*   [ ] T048 [P] [US2] Display typing indicator and handle errors gracefully in chatbot UI in `nextjs/src/components/ChatbotWidget.tsx`
*   [ ] T049 [US2] Ensure chatbot supports both logged-in (persistent history) and anonymous users (session-only) across `fastapi/` and `nextjs/`

## Phase 5: User Story 3 - Sign Up with Technical Background Assessment (P2)

**Story Goal**: As an industry practitioner, I need to create an account and provide my technical background, so the system can personalize content to match my expertise level.
**Independent Test**: Complete signup flow with background questions, verify profile is stored and expertise level is correctly classified.

*   [ ] T050 [P] [US3] Extend `user_profiles` table schema in Neon Postgres with Python level, ROS experience, Linux familiarity, hardware access, learning goal based on `spec.md`
*   [ ] T051 [P] [US3] Implement signup form in `nextjs/src/app/signup/page.tsx` collecting all required background fields
*   [ ] T052 [P] [US3] Implement validation for email format and password strength (min 8 chars, uppercase, lowercase, number) in `nextjs/src/app/signup/page.tsx` and `fastapi/app/api/auth.py`
*   [ ] T053 [P] [US3] Implement expertise classification logic (Beginner, Intermediate, Expert) in `fastapi/app/services/user_service.py`
*   [ ] T054 [P] [US3] Update `POST /api/auth/signup` endpoint to store user profile data and classified expertise level in `fastapi/app/api/auth.py`
*   [ ] T055 [P] [US3] Implement "Forgot Password" functionality in `nextjs/src/app/forgot-password/page.tsx` and `fastapi/app/api/auth.py`
*   [ ] T056 [P] [US3] Implement profile management page for updating background information in `nextjs/src/app/profile/page.tsx`
*   [ ] T057 [US3] Update `PUT /api/user/profile` endpoint to allow users to update their background information in `fastapi/app/api/user.py`

## Phase 6: User Story 4 - Personalize Content Based on Profile (P2)

**Story Goal**: As an industry practitioner, I need to click a "Personalize for Me" button at the start of each chapter, so content adapts to show beginner explanations or advanced insights based on my profile.
**Independent Test**: As beginner vs expert user, personalize same chapter and verify content differences are appropriate.

*   [ ] T058 [P] [US4] Implement "Personalize for Me" button component (logged-in users only) in Docusaurus chapter pages `docusaurus/src/theme/DocItem/Content/index.js`
*   [ ] T059 [P] [US4] Implement FastAPI endpoint `POST /api/personalize/chapter/{chapter_id}` for generating personalized content in `fastapi/app/api/personalization.py`
*   [ ] T060 [P] [US4] Implement FastAPI endpoint `GET /api/personalize/chapter/{chapter_id}/{user_id}` for retrieving cached personalized content in `fastapi/app/api/personalization.py`
*   [ ] T061 [P] [US4] Implement content adaptation logic using OpenAI GPT-4o based on user profile (`expertise_level`, `hardware_access`, `learning_goal`) in `fastapi/app/services/personalization_service.py`
*   [ ] T062 [P] [US4] Implement personalized content caching (24-hour expiration) in Neon Postgres `personalized_content` table `fastapi/app/services/personalization_service.py`
*   [ ] T063 [P] [US4] Implement display of personalized content and toggle between personalized/default in Docusaurus frontend `docusaurus/src/theme/DocItem/Content/index.js`
*   [ ] T064 [P] [US4] Implement displaying clear indicator (e.g., badge "Personalized for Expert") in Docusaurus frontend `docusaurus/src/theme/DocItem/Content/index.js`
*   [ ] T065 [P] [US4] Implement logging of personalization events (`user_id`, `chapter_id`, `expertise_level`, `timestamp`) in `fastapi/app/services/personalization_service.py`
*   [ ] T066 [US4] Ensure beginner personalization adds: analogies, visual diagrams, step-by-step breakdowns, heavily commented code in `fastapi/app/services/personalization_service.py`
*   [ ] T067 [US4] Ensure expert personalization adds: optimization tips, performance benchmarks, advanced configs, research paper links in `fastapi/app/services/personalization_service.py`
*   [ ] T068 [US4] Ensure hardware-specific notes are applied: No GPU → cloud warnings; Jetson → resource constraints; RTX → high-performance tips in `fastapi/app/services/personalization_service.py`

## Phase 7: User Story 5 - Translate Content to Urdu (P2)

**Story Goal**: As a learner, I need to click a button to translate chapter content into Urdu, so I can better understand complex concepts in my native language.
**Independent Test**: Translate 5 chapters, verify technical accuracy and proper RTL rendering.

*   [ ] T069 [P] [US5] Implement "اردو میں پڑھیں" button component at the top of Docusaurus chapter pages `docusaurus/src/theme/DocItem/Content/index.js`
*   [ ] T070 [P] [US5] Implement FastAPI endpoint `POST /api/translate/chapter/{chapter_id}` for translating content to Urdu in `fastapi/app/api/translation.py`
*   [ ] T071 [P] [US5] Implement FastAPI endpoint `GET /api/translate/chapter/{chapter_id}/ur` for retrieving cached Urdu translation in `fastapi/app/api/translation.py`
*   [ ] T072 [P] [US5] Implement content translation logic using OpenAI GPT-4o from English to Urdu in `fastapi/app/services/translation_service.py`
*   [ ] T073 [P] [US5] Ensure technical terms (ROS, URDF, SLAM, Gazebo, Isaac, etc.) are preserved in English during translation in `fastapi/app/services/translation_service.py`
*   [ ] T074 [P] [US5] Ensure code snippets remain in English with Urdu comments translated in `fastapi/app/services/translation_service.py`
*   [ ] T075 [P] [US5] Implement markdown formatting preservation in translations in `fastapi/app/services/translation_service.py`
*   [ ] T076 [P] [US5] Implement translation caching (24-hour expiration) in Neon Postgres `translated_chapters` table `fastapi/app/services/translation_service.py`
*   [ ] T077 [P] [US5] Configure Docusaurus to handle RTL text rendering correctly in `docusaurus/` (CSS/theme adjustments)
*   [ ] T078 [P] [US5] Ensure LaTeX math notation remains unchanged in translations in `fastapi/app/services/translation_service.py`
*   [ ] T079 [P] [US5] Implement translation of navigation elements when content is in Urdu in Docusaurus frontend `docusaurus/src/theme/Navbar/Content/index.js` (and similar files)
*   [ ] T080 [P] [US5] Integrate Urdu language support in RAG chatbot (user can ask/receive responses in Urdu) in `fastapi/app/services/chatbot_service.py`
*   [ ] T081 [US5] Implement glossary with Urdu explanations in `docusaurus/docs/glossary.md`
*   [ ] T082 [US5] Store user language preference (en/ur) in profile or localStorage for Docusaurus `docusaurus/` and FastAPI `fastapi/`

## Phase 8: User Story 6 - Use Claude Code with Subagents (P3)

**Story Goal**: As a developer, I need to leverage Claude Code subagents and skills to accelerate development, so I can build complex features efficiently.
**Independent Test**: Create 3+ custom subagents, use them to build a feature, verify code quality and time savings.

*   [ ] T083 [P] [US6] Create `rag-spec-architect` subagent definition in `.claude/agents/rag-spec-architect.js`
*   [ ] T084 [P] [US6] Create `auth-personalizer` subagent definition in `.claude/agents/auth-personalizer.js`
*   [ ] T085 [P] [US6] Create `docusaurus-book-creator` subagent definition in `.claude/agents/docusaurus-book-creator.js`
*   [ ] T086 [P] [US6] Create `frontend-architect` subagent definition in `.claude/agents/frontend-architect.js`
*   [ ] T087 [P] [US6] Create `deploy-commander` subagent definition in `.claude/agents/deploy-commander.js`
*   [ ] T088 [P] [US6] Create `physical-ai-book-master` subagent definition in `.claude/agents/physical-ai-book-master.js`
*   [ ] T089 [P] [US6] Document agent usage (which agents called for which tasks) in `docs/agent_usage.md`
*   [ ] T090 [P] [US6] Document MCP servers used by each agent in `.claude/agents/*/agent.js`
*   [ ] T091 [P] [US6] Create Custom Skill 1 definition in `.claude/skills/skill-1.js`
*   [ ] T092 [P] [US6] Create Custom Skill 2 definition in `.claude/skills/skill-2.js`
*   [ ] T093 [US6] Implement agent call log tracking in `fastapi/app/services/agent_log_service.py` and store in Neon Postgres `agent_call_logs` table

## Phase 9: Polish & Cross-Cutting Concerns

*   [ ] T094 Implement robust error handling and graceful degradation across all FastAPI endpoints
*   [ ] T095 Implement comprehensive logging for all FastAPI services and integrate with monitoring (e.g., Sentry, CloudWatch)
*   [ ] T096 Review and optimize performance for all critical paths (chatbot, personalization, translation APIs)
*   [ ] T097 Implement unit, integration, and end-to-end tests for all major components
*   [ ] T098 Conduct security review for OWASP Top 10 vulnerabilities across all applications
*   [ ] T099 Optimize Docusaurus build for fast loading and SEO
*   [ ] T100 Create initial content for all Docusaurus chapters (beyond placeholders)

---

## Dependencies

The completion order of user stories is as follows:
1.  **User Story 1** (Browse and Learn from Textbook) - Foundational for all other stories.
2.  **User Story 2** (Get AI-Powered Assistance via RAG Chatbot) - Depends on content from US1.
3.  **User Story 3** (Sign Up with Technical Background Assessment) - Independent, but enables US4.
4.  **User Story 4** (Personalize Content Based on Profile) - Depends on US3 and content from US1.
5.  **User Story 5** (Translate Content to Urdu) - Depends on content from US1.
6.  **User Story 6** (Use Claude Code with Subagents) - Development-focused, can run in parallel with other implementation, but benefits from existing structure.

## Parallel Execution Examples

*   **During User Story 1 (Textbook Structure)**: Tasks T019-T030 can be parallelized.
*   **During User Story 2 (RAG Chatbot)**: Tasks T034-T048 can be parallelized.
*   **During User Story 3 (Authentication)**: Tasks T050-T057 can be parallelized.
*   **During User Story 4 (Personalization)**: Tasks T058-T065 can be parallelized.
*   **During User Story 5 (Translation)**: Tasks T069-T080 can be parallelized.
*   **During User Story 6 (Claude Code Integration)**: Tasks T083-T092 can be parallelized.

## Implementation Strategy

The project will follow an MVP-first, incremental delivery approach.
*   **MVP Scope**: Focus initially on User Story 1 (Docusaurus Textbook) and User Story 2 (RAG Chatbot). These are the core requirements and provide the base 100 points.
*   **Incremental Delivery**: Implement User Stories 3, 4, 5, and 6 as bonus features in subsequent increments, prioritizing P2 features before P3.
*   **Test-Driven Development (TDD)**: Will be applied where appropriate, especially for complex logic in FastAPI services.

---

