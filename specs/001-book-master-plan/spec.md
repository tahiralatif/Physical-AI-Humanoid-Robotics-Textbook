# Complete Hackathon Project Specification: Physical AI & Humanoid Robotics Textbook

**Project Name**: Physical AI & Humanoid Robotics Interactive Textbook
**Hackathon Deadline**: Sunday, Nov 30, 2025 at 06:00 PM
**Created**: 2025-12-07
**Status**: Master Specification
**Total Possible Points**: 100 (base) + 200 (bonus) = 300 points

---

## Executive Summary

This specification covers the complete development of an AI-native interactive textbook for teaching Physical AI & Humanoid Robotics. The project integrates:

1. **Docusaurus-based textbook** (static site deployed to GitHub Pages/Vercel)
2. **Next.js + FastAPI web application** (for chatbot, auth, personalization)
3. **RAG Chatbot** (OpenAI ChatKit SDK + Neon Postgres + Qdrant)
4. **Authentication & Personalization** (Better Auth + user profiling)
5. **Urdu Translation** (OpenAI GPT-4 based translation)
6. **Claude Code + Spec-Kit Plus** (AI-driven development workflow)

---

## Point Distribution Breakdown

| Component | Points | Status |
|-----------|--------|--------|
| **Base Requirements** |
| Docusaurus Book + GitHub Pages Deployment | 40 | Required |
| RAG Chatbot with Text Selection | 60 | Required |
| **Bonus Features** |
| Claude Code Subagents & Skills | 50 | Bonus |
| Better Auth Signup/Signin + Background Collection | 50 | Bonus |
| Per-Chapter Content Personalization | 50 | Bonus |
| Urdu Translation Button | 50 | Bonus |
| **Total Possible** | **300** | |

---

## Technology Stack

### Frontend Stack
- **Book Platform**: Docusaurus 3.x (static site generator)
- **Web Application**: Next.js 14+ (App Router)
- **Styling**: Tailwind CSS + shadcn/ui components
- **Deployment**: Vercel (Next.js) + GitHub Pages (Docusaurus)

### Backend Stack
- **API Framework**: FastAPI (Python)
- **Package Manager**: uv (Python package manager)
- **Database**: Neon Serverless Postgres (conversations, users, profiles)
- **Vector Database**: Qdrant Cloud Free Tier (embeddings)
- **Authentication**: Better Auth (better-auth.com)

### AI/ML Stack
- **Chatbot**: OpenAI ChatKit SDK (for conversational interface)
- **Agents**: OpenAI Agents SDK (for complex workflows)
- **Embeddings**: OpenAI text-embedding-3-small
- **LLM**: OpenAI GPT-4o (for chatbot, translation, personalization)

### Development Tools
- **AI Coding**: Claude Code CLI + Spec-Kit Plus
- **Version Control**: Git + GitHub
- **CI/CD**: GitHub Actions (automated deployment)

---

## User Scenarios & Testing

### User Story 1 - Browse and Learn from Textbook (Priority: P1)

As an **industry practitioner** learning Physical AI, I need to access a well-structured 13-week course textbook, so I can systematically learn ROS 2, Gazebo, NVIDIA Isaac, and humanoid robotics.

**Why this priority**: Core deliverable - the textbook is the foundation (40 points).

**Independent Test**: Navigate through all 13 weeks of content, verify all chapters are accessible and properly formatted.

**Acceptance Scenarios**:

1. **Given** I visit the textbook homepage, **When** I view the dashboard, **Then** I see 4 module cards (ROS 2, Digital Twin, Isaac, VLA) with week ranges and learning outcomes
2. **Given** I am on Week 1, **When** I navigate to Module 1 (ROS 2), **Then** I can access all chapters for Weeks 3-5 with proper prerequisites listed
3. **Given** I need hardware setup guidance, **When** I access the setup section, **Then** I see 3 detailed guides: Digital Twin Workstation (RTX+Ubuntu), Physical AI Edge Kit (Jetson), Cloud-Native Setup (AWS/Azure)
4. **Given** I encounter unfamiliar robotics terms, **When** I use the glossary search, **Then** I get instant definitions with links to relevant chapters

---

### User Story 2 - Get AI-Powered Assistance via RAG Chatbot (Priority: P1)

As an **industry practitioner**, I need to ask questions about textbook content through an embedded chatbot, so I can get immediate clarifications without searching through chapters.

**Why this priority**: Core requirement worth 60 points - demonstrates AI-native learning.

**Independent Test**: Ask 20 diverse questions, verify 90%+ accuracy and proper source attribution.

**Acceptance Scenarios**:

1. **Given** I am reading Chapter 5 (ROS 2 Topics), **When** I ask chatbot "What's the difference between a topic and a service?", **Then** I get accurate answer with citation to specific chapter sections
2. **Given** I select a code snippet about ROS 2 publishers, **When** I ask "Explain this code", **Then** chatbot provides context-specific explanation limited to selected text
3. **Given** I asked 3 questions about URDF, **When** I follow up with "Can you elaborate on the third point?", **Then** chatbot maintains conversation context and provides relevant elaboration
4. **Given** I close browser and return next day, **When** I login, **Then** my conversation history is preserved and accessible

---

### User Story 3 - Sign Up with Technical Background Assessment (Priority: P2)

As an **industry practitioner**, I need to create an account and provide my technical background, so the system can personalize content to match my expertise level.

**Why this priority**: Worth 50 bonus points - enables advanced personalization features.

**Independent Test**: Complete signup flow with background questions, verify profile is stored and expertise level is correctly classified.

**Acceptance Scenarios**:

1. **Given** I am a new user, **When** I click "Sign Up", **Then** I see a form asking: email, password, Python level (Beginner/Intermediate/Expert), ROS experience (None/Basic/Advanced), Linux familiarity, hardware access (None/Jetson/RTX/Cloud), learning goal (Job/Startup/Research)
2. **Given** I have weak Python and no ROS experience, **When** I complete signup, **Then** system classifies me as "Beginner" and shows appropriate welcome message
3. **Given** I am an expert developer with ROS experience, **When** I complete signup, **Then** system classifies me as "Expert" and adjusts content recommendations
4. **Given** I completed signup, **When** I access my profile, **Then** I can view and update my background information anytime

---

### User Story 4 - Personalize Content Based on Profile (Priority: P2)

As an **industry practitioner**, I need to click a "Personalize for Me" button at the start of each chapter, so content adapts to show beginner explanations or advanced insights based on my profile.

**Why this priority**: Worth 50 bonus points - provides tailored learning experience.

**Independent Test**: As beginner vs expert user, personalize same chapter and verify content differences are appropriate.

**Acceptance Scenarios**:

1. **Given** I am a beginner on Chapter 3 (ROS 2 Nodes), **When** I click "Personalize for Me", **Then** I see additional explanations, analogies, visual diagrams, and heavily commented code examples
2. **Given** I am an expert on the same chapter, **When** I click "Personalize for Me", **Then** I see concise explanations, optimization tips, advanced configuration options, and performance benchmarks
3. **Given** I have no GPU (cloud-only), **When** I view personalized Isaac Sim content, **Then** I see prominent warnings about cloud requirements and cost estimates
4. **Given** I have Jetson hardware, **When** I personalize edge computing chapters, **Then** I see resource constraint warnings and lightweight alternatives

---

### User Story 5 - Translate Content to Urdu (Priority: P2)

As a **learner**, I need to click a button to translate chapter content into Urdu, so I can better understand complex concepts in my native language.

**Why this priority**: Worth 50 bonus points - improves accessibility for Urdu speakers.

**Independent Test**: Translate 5 chapters, verify technical accuracy and proper RTL rendering.

**Acceptance Scenarios**:

1. **Given** I am reading Chapter 4 in English, **When** I click "اردو میں پڑھیں" button, **Then** content displays in Urdu with technical terms (ROS, URDF, SLAM) preserved in English
2. **Given** I am viewing Urdu content with code examples, **When** I review code blocks, **Then** code syntax remains in English with Urdu comments
3. **Given** I translated a chapter yesterday, **When** I revisit today, **Then** it still displays in Urdu (preference remembered)
4. **Given** I am viewing Urdu content, **When** I use the chatbot, **Then** I can ask questions in Urdu and get responses in Urdu

---

### User Story 6 - Use Claude Code with Subagents (Priority: P3)

As a **developer**, I need to leverage Claude Code subagents and skills to accelerate development, so I can build complex features efficiently.

**Why this priority**: Worth 50 bonus points - demonstrates advanced AI-assisted development.

**Independent Test**: Create 3+ custom subagents, use them to build a feature, verify code quality and time savings.

**Acceptance Scenarios**:

1. **Given** I need to build RAG system, **When** I invoke `rag-spec-architect` agent, **Then** agent designs vector database schema and retrieval logic
2. **Given** I need auth implementation, **When** I invoke `auth-personalizer` agent, **Then** agent sets up Better Auth with user profiling
3. **Given** I need to create book chapters, **When** I invoke `docusaurus-book-creator` agent, **Then** agent generates structured markdown content
4. **Given** I created custom skills, **When** other agents need similar functionality, **Then** skills are reused across the project

---

### Edge Cases

- **What if Qdrant Free Tier runs out of storage?** → Monitor usage (estimate 10k chunks max), upgrade if needed, provide clear error message to users
- **What if translation API costs exceed budget?** → Implement aggressive caching (24-hour expiration), rate limit translation requests
- **What if user has conflicting profile signals (expert Python but no hardware)?** → Prioritize practical constraints in personalization (show cloud alternatives) while maintaining expert-level explanations
- **What if RAG chatbot gives wrong answer?** → Provide "Report Inaccuracy" button, log feedback, manual review queue
- **What if user skips background assessment?** → Default to "beginner" level, show persistent banner encouraging profile completion for better experience

---

## Functional Requirements

### FR-001: Docusaurus Textbook Requirements

- **FR-001.1**: Book MUST organize content into 4 modules: Module 1 (ROS 2 - Weeks 3-5), Module 2 (Digital Twin - Weeks 6-7), Module 3 (NVIDIA Isaac - Weeks 8-10), Module 4 (VLA & Humanoids - Weeks 11-13)
- **FR-001.2**: Book MUST include Introduction section (Weeks 1-2: Physical AI Foundations)
- **FR-001.3**: Book MUST provide 3 hardware setup guides: RTX Workstation, Jetson Edge Kit, Cloud-Native
- **FR-001.4**: Each module MUST state clear learning outcomes mapping to course objectives
- **FR-001.5**: Book MUST include Capstone Project Guide (autonomous humanoid architecture)
- **FR-001.6**: Book MUST provide 4 assessment guides: ROS 2 package, Gazebo simulation, Isaac perception, Capstone
- **FR-001.7**: Book MUST include reference materials: Glossary (100+ terms with search), Notation Guide, ROS 2 Quick Reference, Troubleshooting Guide
- **FR-001.8**: Navigation MUST use collapsible sidebar organized by modules, supporting access by week (1-13), module (1-4), and topic
- **FR-001.9**: Each chapter MUST declare frontmatter: `estimated_time`, `week`, `module`, `prerequisites`, `learning_objectives`, `sidebar_label`
- **FR-001.10**: Homepage MUST use dashboard layout: 4 module cards, quick links sidebar, recent updates section
- **FR-001.11**: Book MUST be deployed to GitHub Pages or Vercel
- **FR-001.12**: All chapters MUST be properly formatted markdown with syntax-highlighted code blocks

### FR-002: RAG Chatbot Requirements

- **FR-002.1**: System MUST embed chatbot widget on every textbook page (bottom-right, collapsible)
- **FR-002.2**: System MUST use OpenAI ChatKit SDK for chatbot implementation
- **FR-002.3**: System MUST use OpenAI Agents SDK for complex multi-step workflows
- **FR-002.4**: System MUST store textbook content as embeddings in Qdrant Cloud (text-embedding-3-small)
- **FR-002.5**: System MUST store conversation history in Neon Postgres
- **FR-002.6**: System MUST support text selection feature: user highlights text, asks question limited to that context
- **FR-002.7**: System MUST maintain conversation context across minimum 5 turns
- **FR-002.8**: System MUST retrieve top-5 relevant chunks via semantic similarity search
- **FR-002.9**: System MUST cite sources (chapter/section) in responses
- **FR-002.10**: System MUST detect when query cannot be answered from textbook and state explicitly
- **FR-002.11**: System MUST rate-limit queries (50 per user per day)
- **FR-002.12**: System MUST support both logged-in (persistent history) and anonymous users (session-only)
- **FR-002.13**: Chatbot interface MUST show typing indicator and handle errors gracefully
- **FR-002.14**: Backend MUST use FastAPI with endpoints: `/chat`, `/history`, `/clear`
- **FR-002.15**: Backend MUST be managed with uv package manager

### FR-003: Authentication Requirements

- **FR-003.1**: System MUST implement Better Auth for signup/signin
- **FR-003.2**: Signup MUST collect: email, password, Python level, ROS experience, Linux familiarity, hardware access, learning goal
- **FR-003.3**: System MUST classify users into: Beginner (weak Python OR no ROS), Intermediate (good Python AND basic ROS), Expert (expert Python AND advanced ROS)
- **FR-003.4**: System MUST store user profiles in Neon Postgres: user_id, email, expertise_level, python_level, ros_experience, hardware_access, learning_goal
- **FR-003.5**: System MUST validate email format and password strength (min 8 chars, uppercase, lowercase, number)
- **FR-003.6**: System MUST handle session management with secure HTTP-only cookies
- **FR-003.7**: System MUST provide "Forgot Password" functionality
- **FR-003.8**: System MUST provide profile management page for updating background
- **FR-003.9**: Protected routes MUST redirect unauthorized users to login
- **FR-003.10**: System MUST integrate with Next.js App Router middleware for auth checks

### FR-004: Personalization Requirements

- **FR-004.1**: Each chapter MUST display "Personalize for Me" button (logged-in users only)
- **FR-004.2**: Clicking button MUST trigger API call to generate personalized content
- **FR-004.3**: Personalized content MUST include: difficulty-adjusted explanations, hardware-specific warnings, learning goal-aligned examples
- **FR-004.4**: System MUST cache personalized content (24-hour expiration) to reduce API costs
- **FR-004.5**: System MUST clearly indicate content is personalized (e.g., badge "Personalized for Expert")
- **FR-004.6**: System MUST allow toggle between personalized and default content
- **FR-004.7**: System MUST log personalization events: user_id, chapter_id, expertise_level, timestamp
- **FR-004.8**: Personalization API MUST use OpenAI GPT-4o to adapt content based on user profile
- **FR-004.9**: Beginner personalization MUST add: analogies, visual diagrams, step-by-step breakdowns, heavily commented code
- **FR-004.10**: Expert personalization MUST add: optimization tips, performance benchmarks, advanced configs, research paper links
- **FR-004.11**: Hardware-specific notes: No GPU → cloud warnings; Jetson → resource constraints; RTX → high-performance tips

### FR-005: Translation Requirements

- **FR-005.1**: Each chapter MUST display "اردو میں پڑھیں" button at top
- **FR-005.2**: Clicking button MUST translate content from English to Urdu using OpenAI GPT-4o
- **FR-005.3**: System MUST preserve technical terms in English (ROS, URDF, SLAM, Gazebo, Isaac, etc.)
- **FR-005.4**: System MUST keep code snippets in English, translate only comments
- **FR-005.5**: System MUST maintain markdown formatting in translations
- **FR-005.6**: System MUST cache translations (24-hour expiration)
- **FR-005.7**: System MUST handle RTL text rendering correctly
- **FR-005.8**: System MUST preserve LaTeX math notation unchanged
- **FR-005.9**: System MUST translate navigation elements when content is in Urdu
- **FR-005.10**: System MUST support Urdu in RAG chatbot (user can ask/receive responses in Urdu)
- **FR-005.11**: System MUST provide glossary with Urdu explanations (e.g., "Node نوڈ: A process that performs computation")
- **FR-005.12**: System MUST store user language preference (en/ur) in profile or localStorage

### FR-006: Claude Code Integration Requirements

- **FR-006.1**: Project MUST use Claude Code CLI for development
- **FR-006.2**: Project MUST use Spec-Kit Plus for specification-driven development
- **FR-006.3**: Project MUST create minimum 3 custom subagents demonstrating reusable intelligence
- **FR-006.4**: Project MUST document agent usage: which agents called for which tasks
- **FR-006.5**: Project MUST create minimum 2 custom skills that agents can reuse
- **FR-006.6**: Agents MUST be documented in `.claude/agents/` directory
- **FR-006.7**: Skills MUST be documented in `.claude/skills/` directory
- **FR-006.8**: Project MUST maintain agent call log tracking: agent name, task, MCP servers used, outcome

---

## Key Entities & Database Schema

### Neon Postgres Tables

#### users
```sql
CREATE TABLE users (
    user_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW(),
    email_verified BOOLEAN DEFAULT FALSE
);
```

#### user_profiles
```sql
CREATE TABLE user_profiles (
    profile_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(user_id) ON DELETE CASCADE,
    expertise_level VARCHAR(20) CHECK (expertise_level IN ('beginner', 'intermediate', 'expert')),
    python_level VARCHAR(20) CHECK (python_level IN ('beginner', 'intermediate', 'expert')),
    ros_experience VARCHAR(20) CHECK (ros_experience IN ('none', 'basic', 'advanced')),
    linux_familiarity VARCHAR(20) CHECK (linux_familiarity IN ('none', 'basic', 'expert')),
    hardware_access VARCHAR(20) CHECK (hardware_access IN ('none', 'jetson', 'rtx', 'cloud')),
    learning_goal VARCHAR(20) CHECK (learning_goal IN ('job', 'startup', 'research', 'hobby')),
    preferred_language VARCHAR(5) DEFAULT 'en' CHECK (preferred_language IN ('en', 'ur')),
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);
```

#### chat_messages
```sql
CREATE TABLE chat_messages (
    message_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL,
    user_id UUID REFERENCES users(user_id) ON DELETE SET NULL,
    role VARCHAR(20) CHECK (role IN ('user', 'assistant')) NOT NULL,
    content TEXT NOT NULL,
    chapter_context VARCHAR(100),
    selected_text TEXT,
    retrieved_chunk_ids TEXT[], -- Array of chunk IDs from Qdrant
    created_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_session_messages ON chat_messages(session_id, created_at);
CREATE INDEX idx_user_messages ON chat_messages(user_id, created_at);
```

#### conversation_sessions
```sql
CREATE TABLE conversation_sessions (
    session_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(user_id) ON DELETE SET NULL,
    started_at TIMESTAMP DEFAULT NOW(),
    last_message_at TIMESTAMP DEFAULT NOW(),
    message_count INT DEFAULT 0,
    is_active BOOLEAN DEFAULT TRUE
);
```

#### personalized_content
```sql
CREATE TABLE personalized_content (
    content_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(user_id) ON DELETE CASCADE,
    chapter_id VARCHAR(100) NOT NULL,
    personalized_version TEXT NOT NULL,
    adjustments_applied JSONB, -- {"type": "beginner", "added_analogies": 3, "simplified_code": true}
    generated_at TIMESTAMP DEFAULT NOW(),
    cache_expires_at TIMESTAMP NOT NULL
);

CREATE INDEX idx_user_chapter_content ON personalized_content(user_id, chapter_id);
```

#### translated_chapters
```sql
CREATE TABLE translated_chapters (
    translation_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chapter_id VARCHAR(100) NOT NULL,
    source_content_hash VARCHAR(64) NOT NULL, -- SHA256 of source English content
    translated_content TEXT NOT NULL,
    language_code VARCHAR(5) DEFAULT 'ur',
    technical_terms_preserved TEXT[], -- ["ROS", "URDF", "SLAM"]
    generated_at TIMESTAMP DEFAULT NOW(),
    cache_expires_at TIMESTAMP NOT NULL
);

CREATE INDEX idx_chapter_translation ON translated_chapters(chapter_id, language_code);
```

#### agent_call_logs
```sql
CREATE TABLE agent_call_logs (
    log_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    agent_name VARCHAR(100) NOT NULL,
    task_description TEXT NOT NULL,
    mcp_servers_used TEXT[], -- ["qdrant", "postgres", "context7"]
    outcome VARCHAR(20) CHECK (outcome IN ('success', 'failure', 'partial')),
    execution_time_ms INT,
    created_at TIMESTAMP DEFAULT NOW()
);
```

### Qdrant Collection Schema

#### textbook_chunks
```python
{
    "collection_name": "textbook_chunks",
    "vectors": {
        "size": 1536,  # text-embedding-3-small dimension
        "distance": "Cosine"
    },
    "payload_schema": {
        "chunk_id": "string",
        "chapter_id": "string",
        "section_title": "string",
        "content": "string",  # The actual text
        "metadata": {
            "week": "integer",
            "module": "integer",
            "page_number": "integer",
            "estimated_reading_time_min": "integer"
        }
    }
}
```

---

## API Endpoints

### FastAPI Backend Endpoints

#### Authentication (Better Auth handles these, integrated with FastAPI)
- `POST /api/auth/signup` - Create new user account
- `POST /api/auth/signin` - Login user
- `POST /api/auth/signout` - Logout user
- `GET /api/auth/session` - Get current session
- `POST /api/auth/forgot-password` - Send password reset email
- `PUT /api/user/profile` - Update user profile

#### RAG Chatbot
- `POST /api/chat` - Send message to chatbot
  ```json
  {
    "session_id": "uuid",
    "message": "What is ROS 2?",
    "chapter_context": "week-3-ros2-fundamentals",
    "selected_text": "optional highlighted text"
  }
  ```
- `GET /api/chat/history/{session_id}` - Get conversation history
- `DELETE /api/chat/history/{session_id}` - Clear conversation history
- `GET /api/chat/sessions` - Get user's conversation sessions (requires auth)

#### Personalization
- `POST /api/personalize/chapter/{chapter_id}` - Generate personalized content
  ```json
  {
    "user_id": "uuid",
    "expertise_level": "expert",
    "hardware_access": "jetson"
  }
  ```
- `GET /api/personalize/chapter/{chapter_id}/{user_id}` - Get cached personalized content

#### Translation
- `POST /api/translate/chapter/{chapter_id}` - Translate chapter to Urdu
  ```json
  {
    "language": "ur",
    "preserve_terms": ["ROS", "URDF", "SLAM"]
  }
  ```
- `GET /api/translate/chapter/{chapter_id}/ur` - Get cached Urdu translation

#### Analytics
- `GET /api/analytics/usage` - Get usage statistics (admin only)
- `POST /api/analytics/feedback` - Submit user feedback
  ```json
  {
    "type": "translation_issue",
    "chapter_id": "week-3-ros2-fundamentals",
    "description": "Technical term incorrectly translated"
  }
  ```

---

## Success Criteria

### Base Requirements (100 points)

- **SC-001**: Docusaurus book deployed to GitHub Pages/Vercel with all 13 weeks of structured content (40 points)
- **SC-002**: All 4 modules (ROS 2, Digital Twin, Isaac, VLA) fully documented with learning outcomes and prerequisites
- **SC-003**: RAG chatbot embedded on every page, answers 90% of textbook questions accurately (60 points)
- **SC-004**: Text selection feature works correctly 95% of time (responses limited to selected content)
- **SC-005**: Conversation history persists for logged-in users with 100% retention
- **SC-006**: Chatbot maintains context across 5+ turns with 90% accuracy
- **SC-007**: Average chatbot response time under 3 seconds for 95% of queries

### Bonus Requirements (200 points)

#### Claude Code Subagents & Skills (50 points)
- **SC-008**: Minimum 3 custom subagents created and used in project development
- **SC-009**: Minimum 2 custom skills demonstrating reusable intelligence
- **SC-010**: Agent call logs document which agents were used for which tasks
- **SC-011**: Clear documentation of MCP servers used by each agent

#### Better Auth (50 points)
- **SC-012**: Signup/signin working with Better Auth integration
- **SC-013**: Background assessment collects all required fields (Python, ROS, hardware, etc.)
- **SC-014**: 95% of users complete background assessment during signup
- **SC-015**: User expertise level correctly classified based on assessment
- **SC-016**: Profile management page allows users to update background

#### Personalization (50 points)
- **SC-017**: "Personalize for Me" button appears on all chapters for logged-in users
- **SC-018**: Personalized content accurately reflects expertise level (beginner vs expert verified on 10 sample chapters)
- **SC-019**: Hardware-specific warnings appear for 100% of users with constraints
- **SC-020**: Personalization loads in under 2 seconds with caching (90% of requests)

#### Urdu Translation (50 points)
- **SC-021**: "اردو میں پڑھیں" button appears on all chapters
- **SC-022**: 95% of technical terms correctly preserved in English within Urdu translations
- **SC-023**: 100% of code snippets remain syntactically valid after translation
- **SC-024**: RTL rendering works correctly on all major browsers
- **SC-025**: Translation quality score (automated checks) above 90% for all chapters

---

## Agent & MCP Orchestration Guide

### Custom Subagents (3+ required for 50 bonus points)

#### 1. rag-spec-architect
**Purpose**: Design and implement RAG chatbot system
**Responsibilities**:
- Design Qdrant vector database schema
- Implement semantic search retrieval logic
- Develop FastAPI endpoints for chatbot
- Integrate OpenAI ChatKit SDK
- Handle conversation context management

**When to call**: 
- Designing RAG architecture
- Implementing vector search
- Building chatbot API endpoints
- Debugging retrieval accuracy issues

**MCP Servers Used**:
- Qdrant MCP (vector operations)
- PostgreSQL MCP (conversation storage)
- Context7 MCP (ChatKit SDK docs)

---

#### 2. auth-personalizer
**Purpose**: Implement authentication and content personalization
**Responsibilities**:
- Set up Better Auth integration
- Design user profile schema
- Implement background assessment logic
- Build personalization engine
- Create expertise classification algorithm

**When to call**:
- Setting up Better Auth
- Building signup/signin flows
- Implementing user profiling
- Creating personalization API
- Designing adaptive content logic

**MCP Servers Used**:
- PostgreSQL MCP (user accounts, profiles)
- Context7 MCP (Better Auth docs)

---

#### 3. docusaurus-book-creator
**Purpose**: Create comprehensive technical book content
**Responsibilities**:
- Structure course content into 13 weeks
- Write chapter markdown with proper frontmatter
- Create hardware setup guides
- Develop assessment rubrics
- Build glossary and reference materials

**When to call**:
- Creating new chapters
- Structuring course modules
- Writing technical documentation
- Formatting markdown content
- Organizing learning materials

**MCP Servers Used**:
- Context7 MCP (Docusaurus docs)
- Web Search MCP (research robotics topics)

---

#### 4. frontend-architect
**Purpose**: Design and implement Next.js UI components
**Responsibilities**:
- Build chatbot widget with shadcn/ui
- Create authentication forms
- Design personalization button component
- Implement translation toggle
- Develop user dashboard

**When to call**:
- Creating new UI components
- Styling with Tailwind CSS
- Integrating shadcn/ui components
- Implementing responsive layouts
- Building interactive features

**MCP Servers Used**:
- Context7 MCP (Next.js, shadcn/ui docs)
- GitHub MCP (component examples)

---

#### 5. deploy-commander
**Purpose**: Handle deployment and DevOps tasks
**Responsibilities**:
- Configure Vercel deployment
- Set up GitHub Actions CI/CD
- Manage environment variables
- Optimize build performance
- Monitor deployment health

**When to call**:
- Deploying to production
- Configuring CI/CD pipelines
- Setting up environment secrets
- Troubleshooting deployment issues
- Optimizing build times

**MCP Servers Used**:
- GitHub MCP (repo configuration)
- Context7 MCP (Vercel docs)

---

#### 6. physical-ai-book-master
**Purpose**: Expert in Physical AI curriculum and robotics content
**Responsibilities**:
- Ensure technical accuracy of robotics concepts
- Structure ROS 2, Gazebo, Isaac content
- Design hands-on exercises
- Create capstone project architecture
- Validate hardware requirements

**When to call**:
- Writing robotics-specific content
- Validating technical accuracy
- Designing lab exercises
- Structuring curriculum progression
- Reviewing capstone requirements

**MCP Servers Used**:
- Web Search MCP (robotics research)
- Context7 MCP (ROS 2, Isaac docs)

---

### Custom Skills 
- create custome skills