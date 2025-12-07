---
id: 2
title: sp.clarify Failed - Not on Feature Branch
stage: general
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.clarify # Feature Specification: Physical AI Textboo animation
- [ ] Tagline clearly states value proposition: "From Digital Brain to Physical Body"
- [ ] Prominent "Read Book" button (glassmorphism style) that stands out
- [ ] 4 module cards displayed in grid layout with icons, week ranges, and descriptions
- [ ] Stats section showing "13 Weeks · 4 Modules · 1 Capstone Project"
- [ ] Features grid highlighting 6 key features (RAG chatbot, personalization, etc.)
- [ ] Dark/Light mode toggle with smooth transition
- [ ] Mobile-responsive design (320px to 1440px+ breakpoints)
- [ ] Page load time under 3 seconds
- [ ] Smooth scroll animations using Framer Motion

**Success Metrics**:
- Lighthouse performance score ≥ 90
- Accessibility score ≥ 95
- First Contentful Paint < 1.5s
- Cumulative Layout Shift < 0.1

---

### User Story 2: "Read Book" Navigation (Priority: P0)

**As a** learner ready to start the course
**I want** clicking "Read Book" to take me to a clean, organized book dashboard
**So that** I can immediately start reading without confusion

**Acceptance Criteria**:
- [ ] Button navigates to `/book/intro` route
- [ ] Transition animation between landing page and book (fade or slide)
- [ ] No page reload (Next.js client-side navigation)
- [ ] Loading indicator if content takes > 500ms to render
- [ ] Browser back button returns to landing page smoothly

---

### User Story 3: Book Dashboard Structure (Priority: P0)

**As a** learner in the book interface
**I want** a clear sidebar navigation showing all modules, weeks, and chapters
**So that** I can understand the course structure and jump to any section

**Acceptance Criteria**:
- [ ] Left sidebar with nested, collapsible navigation tree
- [ ] Navigation structure:
  - 📖 Introduction (Weeks 1-2)
  - 🤖 Module 1: ROS 2 (Weeks 3-5)
  - 🌐 Module 2: Digital Twin (Weeks 6-7)
  - 🚀 Module 3: NVIDIA Isaac (Weeks 8-10)
  - 🧠 Module 4: VLA (Weeks 11-13)
  - ⚙️ Hardware Setup
  - 📝 Assessments
  - 📚 References
- [ ] Current chapter highlighted with colored accent
- [ ] Hover effects on sidebar items
- [ ] Icons for each module (consistent visual language)
- [ ] "Back to Dashboard" button always visible
- [ ] Sidebar collapsible on mobile (hamburger menu)

---

### User Story 4: Chapter Reading Experience (Priority: P1)

**As a** learner reading a chapter
**I want** clean typography, proper spacing, and clear visual hierarchy
**So that** I can focus on learning without visual strain

**Acceptance Criteria**:
- [ ] Chapter header shows:
  - Week number badge (e.g., "Week 3")
  - Module badge (e.g., "Module 1")
  - Estimated time (e.g., "⏱️ 4.5 hours")
  - Difficulty tag with color (Beginner: Green, Intermediate: Yellow, Advanced: Red)
  - Prerequisites as clickable chips
- [ ] Markdown content rendered with proper styling:
  - Headers (H1-H6) with consistent hierarchy
  - Paragraphs with readable line height (1.7)
  - Lists (ordered/unordered) with proper indentation
  - Blockquotes with left border accent
  - Tables with alternating row colors
- [ ] Code blocks with:
  - Syntax highlighting (language-specific)
  - Copy button (top-right corner)
  - Line numbers (optional, configurable)
  - Language badge (e.g., "Python", "Bash")
- [ ] Images:
  - Responsive (max-width: 100%)
  - Alt text for accessibility
  - Optional zoom on click
  - Caption support
- [ ] Right sidebar with auto-generated Table of Contents (from H2/H3 headings)
- [ ] Scroll progress indicator (top of page)

---

### User Story 5: Responsive Design (Priority: P0)

**As a** learner using any device (phone, tablet, laptop, desktop)
**I want** the textbook to be fully functional and beautiful on my screen size
**So that** I can learn comfortably wherever I am

**Acceptance Criteria**:
- [ ] Mobile (320px - 767px):
  - Hamburger menu for sidebar
  - Single column layout
  - Touch-friendly buttons (min 44px)
  - Module cards stack vertically
- [ ] Tablet (768px - 1023px):
  - Sidebar collapses to icons only (expandable)
  - 2-column grid for module cards
- [ ] Desktop (1024px+):
  - Full sidebar visible
  - 4-column grid for module cards
  - Right TOC sidebar visible
- [ ] Large Desktop (1440px+):
  - Maximum content width: 1280px (centered)
  - Generous whitespace

---

## 📋 Functional Requirements

### FR-001: Landing Page Structure
**Priority**: P0
**Description**: Landing page MUST include all sections in specified order

**Sections (Top to Bottom)**:
1. **Navbar**:
   - Logo/Title (left)
   - Navigation links: Home, Book, About (center)
   - Theme toggle (right)
   - Sign In button (right) - placeholder for Phase 2

2. **Hero Section**:
   - Full viewport height (100vh)
   - Animated 3D robot OR high-quality Lottie animation
   - Tagline: "From Digital Brain to Physical Body"
   - Subtitle: "Master Physical AI & Humanoid Robotics in 13 Weeks"
   - Primary CTA: "Read Book" button (large, prominent)
   - Stats badges: "13 Weeks · 4 Modules · 1 Capstone Project"

3. **Module Cards Section**:
   - Section title: "Four Modules, One Vision"
   - 4 cards in grid (responsive: 1 col mobile, 2 col tablet, 4 col desktop)
   - Each card shows:
     - Module icon
     - Module title
     - Week range
     - Short description (1-2 sentences)
     - "Start Module" button (secondary CTA)

4. **Features Grid**:
   - Section title: "Why This Textbook?"
   - 6 feature cards (2 cols mobile, 3 cols desktop):
     - ✅ AI-Powered RAG Chatbot
     - ✅ Personalized Learning Paths
     - ✅ Urdu Translation Support
     - ✅ Interactive Code Examples
     - ✅ Real Hardware Guides
    adows on hover (buttons, cards)

---

### DR-002: Animation Guidelines
**Priority**: P1
**Description**: Animations MUST enhance UX, not distract

**Animation Principles**:
- Duration: 200-500ms (never > 1s)
- Easing: Use natural curves (ease-out for entrances, ease-in for exits)
- Purpose: Every animation serves a purpose (feedback, guidance, delight)
- Performance: No jank (60fps minimum)
- Respect prefers-reduced-motion (disable animations if user prefers)

**Common Animations**:
- Fade in/out (opacity transitions)
- Slide up/down (y-axis transitions)
- Scale (for hover effects on buttons/cards)
- Rotate (for icons like chevrons in accordion)
- Skeleton loading (for async content)

---

### DR-003: Component Library (shadcn/ui)
**Priority**: P0
**Description**: Use shadcn/ui for consistent, accessible components

**Core Components Needed**:
- Button (primary, secondary, outline, ghost)
- Card (for module cards, feature cards)
- Badge (for week numbers, difficulty tags)
- Dialog (for future modals)
- Dropdown Menu (for future user menu)
- Input (for future search)
- Separator (horizontal rules)
- Skeleton (loading states)
- Tooltip (for icon explanations)
- Scroll Area (for sidebar)

**Customization**:
- All components styled with Tailwind
- Colors match design system
- Variants for different use cases

---

## 🏗️ Technical Architecture

### TA-001: Project Structure
**Priority**: P0
**Description**: Clear, scalable folder organization

**Directory Structure**:
```
/
├── app/                    (Next.js App Router)
│   ├── layout.tsx          (Root layout with Navbar, theme provider)
│   ├── page.tsx            (Landing page)
│   ├── book/
│   │   ├── layout.tsx      (Book layout with sidebar)
│   │   ├── [module]/
│   │   │   └── [chapter]/
│   │   │       └── page.tsx
│   │   └── intro/
│   │       └── page.tsx
│   └── globals.css         (Global styles, Tailwind imports)
│
├── components/
│   ├── landing/            (Landing page components)
│   │   ├── HeroSection.tsx
│   │   ├── ModuleCards.tsx
│   │   ├── FeaturesGrid.tsx
│   │   └── AnimatedRobot.tsx
│   ├── book/               (Book reader components)
│   │   ├── Sidebar.tsx
│   │   ├── ChapterHeader.tsx
│   │   ├── MarkdownRenderer.tsx
│   │   ├── CodeBlock.tsx
│   │   └── TableOfContents.tsx
│   ├── ui/                 (shadcn/ui components)
│   │   ├── button.tsx
│   │   ├── card.tsx
│   │   └── ...
│   ├── Navbar.tsx
│   └── ThemeToggle.tsx
│
├── content/                (MDX files for book chapters)
│   ├── intro/
│   │   ├── what-is-physical-ai.mdx
│   │   └── course-overview.mdx
│   ├── module-1/
│   │   ├── week3-nodes-topics.mdx
│   │   └── ...
│   └── ...
│
├── lib/                    (Utility functions)
│   ├── mdx.ts              (MDX processing)
│   ├── utils.ts            (General utilities)
│   └── constants.ts        (Site config, nav structure)
│
├── public/                 (Static assets)
│   ├── images/
│   ├── icons/
│   └── robots/
│       └── humanoid.json   (Lottie animation)
│
└── styles/
    └── globals.css         (Tailwind directives, custom CSS)
```

---

### TA-002: Technology Stack
**Priority**: P0
**Description**: Specific versions and libraries

**Core Stack**:
- Next.js 14.2+ (App Router, RSC)
- TypeScript 5.3+
- Tailwind CSS 3.4+
- React 18+

**UI Libraries**:
- shadcn/ui (component primitives)
- Framer Motion (animations)
- Lucide React (icons)
- next-themes (theme management)

**Content Processing**:
- @next/mdx or next-mdx-remote (MDX rendering)
- rehype-highlight (syntax highlighting)
- remark-gfm (GitHub Flavored Markdown)
- gray-matter (frontmatter parsing)

**3D/Animation** (choose one):
- React Three Fiber + Three.js (for 3D robot)
- OR Lottie React (for simpler animation)

---

### TA-003: Routing Strategy
**Priority**: P0
**Description**: Next.js 14 App Router file-based routing

**Route Structure**:
```
/                           → Landing page
/book/intro                 → Introduction
/book/module-1/week3        → Module 1, Week 3
/book/module-1/week4        → Module 1, Week 4
...
/book/hardware-setup        → Hardware guides
/book/assessments           → Assessment section
/book/references/glossary   → Glossary
```

**Route Conventions**:
- Use kebab-case for URLs (e.g., `/book/hardware-setup`)
- Module folders named: `module-1`, `module-2`, etc.
- Chapter files use descriptive names: `week3-nodes-topics.mdx`

---

### TA-004: MDX Content Schema
**Priority**: P0
**Description**: Frontmatter metadata structure for chapters

**Frontmatter Fields**:
```yaml
---
title: "Week 3: ROS 2 Nodes and Topics"
week: 3
module: 1
estimated_time: 4.5
difficulty: "Beginner"
prerequisites:
  - "Python basics"
  - "Linux terminal familiarity"
learning_objectives:
  - "Understand ROS 2 node architecture"
  - "Create publishers and subscribers"
  - "Debug node communication issues"
sidebar_label: "Week 3: Nodes & Topics"
---
```

**Field Specifications**:
- `title` (string, required): Full chapter title
- `week` (number, required): Week number (1-13)
- `module` (number, required): Module number (1-4)
- `estimated_time` (number, required): Hours to complete
- `difficulty` (enum, required): "Beginner" | "Intermediate" | "Advanced"
- `prerequisites` (array, optional): List of prerequisite topics
- `learning_objectives` (array, required): List of learning goals
- `sidebar_label` (string, required): Shortened label for sidebar

---

## ✅ Acceptance Criteria

### AC-001: Landing Page
- [ ] Hero section displays with animation
- [ ] "Read Book" button is visually prominent
- [ ] 4 module cards render in responsive grid
- [ ] Features grid displays all 6 features
- [ ] Dark/Light mode toggle works smoothly
- [ ] All sections visible without horizontal scroll on mobile
- [ ] Lighthouse performance score ≥ 90

### AC-002: Book Navigation
- [ ] Sidebar shows complete content structure
- [ ] Clicking a module expands/collapses children
- [ ] Clicking a chapter navigates to that chapter
- [ ] Current chapter is highlighted
- [ ] Sidebar is collapsible on mobile
- [ ] Breadcrumb trail shows current location

### AC-003: Chapter Reading
- [ ] Chapter header displays all metadata correctly
- [ ] Markdown renders with proper styling
- [ ] Code blocks have syntax highlighting
- [ ] Copy button works on code blocks
- [ ] Images are responsive
- [ ] Table of Contents auto-generates from headings

### AC-004: Responsive Design
- [ ] Layout works on 320px width (small mobile)
- [ ] Layout works on 768px width (tablet)
- [ ] Layout works on 1024px width (desktop)
- [ ] Layout works on 1440px+ width (large desktop)
- [ ] Touch targets are min 44px on mobile
- [ ] No content cut off on any screen size

### AC-005: Performance
- [ ] Initial page load < 3 seconds
- [ ] Navigation between pages < 500ms
- [ ] No layout shift during load (CLS < 0.1)
- [ ] Images lazy load below fold
- [ ] No console errors or warnings

### AC-006: Accessibility
- [ ] Semantic HTML used throughout
- [ ] Keyboard navigation fully supported
- [ ] ARIA attributes for dynamic content
- [ ] Color contrast meets WCAG AA standards
- [ ] Focus indicators visible
- [ ] Screen reader compatibility tested

---

## 🗓️ Phase 1 Tasks (High-level)

- [ ] Set up Next.js project with Tailwind CSS and TypeScript
- [ ] Implement Landing Page (Hero, Modules, Features, Navbar, Theme toggle)
- [ ] Implement Book Layout (Sidebar, Chapter Header, Markdown Renderer)
- [ ] Add 3 placeholder MDX chapters
- [ ] Ensure responsive design across all breakpoints
- [ ] Integrate Framer Motion for animations
- [ ] Implement basic routing for book sections
- [ ] Integrate shadcn/ui components
- [ ] Performance and accessibility optimization
- [ ] Browser compatibility testing
- [ ] Final polish and bug fixes
- [ ] Prepare for Phase 2 (RAG chatbot)

---

## 🧪 Testing Strategy

**Manual Testing**:
- [ ] Test on Chrome, Firefox, Safari, Edge
- [ ] Test on iPhone (Safari)
- [ ] Test on Android (Chrome)
- [ ] Test on tablet (iPad)
- [ ] Test keyboard navigation
- [ ] Test with screen reader (NVDA/VoiceOver)

**Automated Testing**:
- [ ] Lighthouse CI in GitHub Actions
- [ ] Broken link checker
- [ ] Image optimization checker
- [ ] Bundle size monitoring

**User Testing**:
- [ ] Show to 2-3 peers for feedback
- [ ] Note pain points and confusion
- [ ] Iterate based on feedback

---

## 📖 Resources to Consult (via Context7)

**MUST research before implementing**:
- Next.js 14 App Router documentation
- Tailwind CSS v3 documentation
- shadcn/ui component documentation
- MDX documentation
- Framer Motion API reference
- Web accessibility guidelines (WCAG)
- React best practices
- TypeScript patterns

**Agent calls required**:
```
@context7-docs-analyst "Get Next.js 14 App Router file-based routing patterns"
@context7-docs-analyst "Get Tailwind CSS responsive design utilities"
@context7-docs-analyst "Get shadcn/ui Card component implementation"
@context7-docs-analyst "Get Framer Motion animation variants patterns"
@context7-docs-analyst "Get MDX rendering with syntax highlighting"
```

---

## 🎯 Definition of Done

**Phase 1 is complete when**:
- [ ] Landing page is live and visually stunning
- [ ] "Read Book" button navigates to book dashboard
- [ ] Book sidebar shows complete content structure
- [ ] At least 3 placeholder chapters render correctly
- [ ] Dark/Light mode works flawlessly
- [ ] All responsive breakpoints tested and polished
- [ ] Lighthouse scores meet targets (Performance ≥ 90, Accessibility ≥ 95)
- [ ] No console errors or warnings
- [ ] Code reviewed and approved
- [ ] Deployed to development environment
- [ ] RESOURCES.md updated with all work done

---

**Version**: 1.0
**Last Updated**: December 6, 2025
**Next Review**: After Phase 1 completion
---

## Prompt

# Feature Specification: Physical AI Textboo animation
- [ ] Tagline clearly states value proposition: "From Digital Brain to Physical Body"
- [ ] Prominent "Read Book" button (glassmorphism style) that stands out
- [ ] 4 module cards displayed in grid layout with icons, week ranges, and descriptions
- [ ] Stats section showing "13 Weeks · 4 Modules · 1 Capstone Project"
- [ ] Features grid highlighting 6 key features (RAG chatbot, personalization, etc.)
- [ ] Dark/Light mode toggle with smooth transition
- [ ] Mobile-responsive design (320px to 1440px+ breakpoints)
- [ ] Page load time under 3 seconds
- [ ] Smooth scroll animations using Framer Motion

**Success Metrics**:
- Lighthouse performance score ≥ 90
- Accessibility score ≥ 95
- First Contentful Paint < 1.5s
- Cumulative Layout Shift < 0.1

---

### User Story 2: "Read Book" Navigation (Priority: P0)

**As a** learner ready to start the course
**I want** clicking "Read Book" to take me to a clean, organized book dashboard
**So that** I can immediately start reading without confusion

**Acceptance Criteria**:
- [ ] Button navigates to `/book/intro` route
- [ ] Transition animation between landing page and book (fade or slide)
- [ ] No page reload (Next.js client-side navigation)
- [ ] Loading indicator if content takes > 500ms to render
- [ ] Browser back button returns to landing page smoothly

---

### User Story 3: Book Dashboard Structure (Priority: P0)

**As a** learner in the book interface
**I want** a clear sidebar navigation showing all modules, weeks, and chapters
**So that** I can understand the course structure and jump to any section

**Acceptance Criteria**:
- [ ] Left sidebar with nested, collapsible navigation tree
- [ ] Navigation structure:
  - 📖 Introduction (Weeks 1-2)
  - 🤖 Module 1: ROS 2 (Weeks 3-5)
  - 🌐 Module 2: Digital Twin (Weeks 6-7)
  - 🚀 Module 3: NVIDIA Isaac (Weeks 8-10)
  - 🧠 Module 4: VLA (Weeks 11-13)
  - ⚙️ Hardware Setup
  - 📝 Assessments
  - 📚 References
- [ ] Current chapter highlighted with colored accent
- [ ] Hover effects on sidebar items
- [ ] Icons for each module (consistent visual language)
- [ ] "Back to Dashboard" button always visible
- [ ] Sidebar collapsible on mobile (hamburger menu)

---

### User Story 4: Chapter Reading Experience (Priority: P1)

**As a** learner reading a chapter
**I want** clean typography, proper spacing, and clear visual hierarchy
**So that** I can focus on learning without visual strain

**Acceptance Criteria**:
- [ ] Chapter header shows:
  - Week number badge (e.g., "Week 3")
  - Module badge (e.g., "Module 1")
  - Estimated time (e.g., "⏱️ 4.5 hours")
  - Difficulty tag with color (Beginner: Green, Intermediate: Yellow, Advanced: Red)
  - Prerequisites as clickable chips
- [ ] Markdown content rendered with proper styling:
  - Headers (H1-H6) with consistent hierarchy
  - Paragraphs with readable line height (1.7)
  - Lists (ordered/unordered) with proper indentation
  - Blockquotes with left border accent
  - Tables with alternating row colors
- [ ] Code blocks with:
  - Syntax highlighting (language-specific)
  - Copy button (top-right corner)
  - Line numbers (optional, configurable)
  - Language badge (e.g., "Python", "Bash")
- [ ] Images:
  - Responsive (max-width: 100%)
  - Alt text for accessibility
  - Optional zoom on click
  - Caption support
- [ ] Right sidebar with auto-generated Table of Contents (from H2/H3 headings)
- [ ] Scroll progress indicator (top of page)

---

### User Story 5: Responsive Design (Priority: P0)

**As a** learner using any device (phone, tablet, laptop, desktop)
**I want** the textbook to be fully functional and beautiful on my screen size
**So that** I can learn comfortably wherever I am

**Acceptance Criteria**:
- [ ] Mobile (320px - 767px):
  - Hamburger menu for sidebar
  - Single column layout
  - Touch-friendly buttons (min 44px)
  - Module cards stack vertically
- [ ] Tablet (768px - 1023px):
  - Sidebar collapses to icons only (expandable)
  - 2-column grid for module cards
- [ ] Desktop (1024px+):
  - Full sidebar visible
  - 4-column grid for module cards
  - Right TOC sidebar visible
- [ ] Large Desktop (1440px+):
  - Maximum content width: 1280px (centered)
  - Generous whitespace

---

## 📋 Functional Requirements

### FR-001: Landing Page Structure
**Priority**: P0
**Description**: Landing page MUST include all sections in specified order

**Sections (Top to Bottom)**:
1. **Navbar**:
   - Logo/Title (left)
   - Navigation links: Home, Book, About (center)
   - Theme toggle (right)
   - Sign In button (right) - placeholder for Phase 2

2. **Hero Section**:
   - Full viewport height (100vh)
   - Animated 3D robot OR high-quality Lottie animation
   - Tagline: "From Digital Brain to Physical Body"
   - Subtitle: "Master Physical AI & Humanoid Robotics in 13 Weeks"
   - Primary CTA: "Read Book" button (large, prominent)
   - Stats badges: "13 Weeks · 4 Modules · 1 Capstone Project"

3. **Module Cards Section**:
   - Section title: "Four Modules, One Vision"
   - 4 cards in grid (responsive: 1 col mobile, 2 col tablet, 4 col desktop)
   - Each card shows:
     - Module icon
     - Module title
     - Week range
     - Short description (1-2 sentences)
     - "Start Module" button (secondary CTA)

4. **Features Grid**:
   - Section title: "Why This Textbook?"
   - 6 feature cards (2 cols mobile, 3 cols desktop):
     - ✅ AI-Powered RAG Chatbot
     - ✅ Personalized Learning Paths
     - ✅ Urdu Translation Support
     - ✅ Interactive Code Examples
     - ✅ Real Hardware Guides
    adows on hover (buttons, cards)

---

### DR-002: Animation Guidelines
**Priority**: P1
**Description**: Animations MUST enhance UX, not distract

**Animation Principles**:
- Duration: 200-500ms (never > 1s)
- Easing: Use natural curves (ease-out for entrances, ease-in for exits)
- Purpose: Every animation serves a purpose (feedback, guidance, delight)
- Performance: No jank (60fps minimum)
- Respect prefers-reduced-motion (disable animations if user prefers)

**Common Animations**:
- Fade in/out (opacity transitions)
- Slide up/down (y-axis transitions)
- Scale (for hover effects on buttons/cards)
- Rotate (for icons like chevrons in accordion)
- Skeleton loading (for async content)

---

### DR-003: Component Library (shadcn/ui)
**Priority**: P0
**Description**: Use shadcn/ui for consistent, accessible components

**Core Components Needed**:
- Button (primary, secondary, outline, ghost)
- Card (for module cards, feature cards)
- Badge (for week numbers, difficulty tags)
- Dialog (for future modals)
- Dropdown Menu (for future user menu)
- Input (for future search)
- Separator (horizontal rules)
- Skeleton (loading states)
- Tooltip (for icon explanations)
- Scroll Area (for sidebar)

**Customization**:
- All components styled with Tailwind
- Colors match design system
- Variants for different use cases

---

## 🏗️ Technical Architecture

### TA-001: Project Structure
**Priority**: P0
**Description**: Clear, scalable folder organization

**Directory Structure**:
```
/
├── app/                    (Next.js App Router)
│   ├── layout.tsx          (Root layout with Navbar, theme provider)
│   ├── page.tsx            (Landing page)
│   ├── book/
│   │   ├── layout.tsx      (Book layout with sidebar)
│   │   ├── [module]/
│   │   │   └── [chapter]/
│   │   │       └── page.tsx
│   │   └── intro/
│   │       └── page.tsx
│   └── globals.css         (Global styles, Tailwind imports)
│
├── components/
│   ├── landing/            (Landing page components)
│   │   ├── HeroSection.tsx
│   │   ├── ModuleCards.tsx
│   │   ├── FeaturesGrid.tsx
│   │   └── AnimatedRobot.tsx
│   ├── book/               (Book reader components)
│   │   ├── Sidebar.tsx
│   │   ├── ChapterHeader.tsx
│   │   ├── MarkdownRenderer.tsx
│   │   ├── CodeBlock.tsx
│   │   └── TableOfContents.tsx
│   ├── ui/                 (shadcn/ui components)
│   │   ├── button.tsx
│   │   ├── card.tsx
│   │   └── ...
│   ├── Navbar.tsx
│   └── ThemeToggle.tsx
│
├── content/                (MDX files for book chapters)
│   ├── intro/
│   │   ├── what-is-physical-ai.mdx
│   │   └── course-overview.mdx
│   ├── module-1/
│   │   ├── week3-nodes-topics.mdx
│   │   └── ...
│   └── ...
│
├── lib/                    (Utility functions)
│   ├── mdx.ts              (MDX processing)
│   ├── utils.ts            (General utilities)
│   └── constants.ts        (Site config, nav structure)
│
├── public/                 (Static assets)
│   ├── images/
│   ├── icons/
│   └── robots/
│       └── humanoid.json   (Lottie animation)
│
└── styles/
    └── globals.css         (Tailwind directives, custom CSS)
```

---

### TA-002: Technology Stack
**Priority**: P0
**Description**: Specific versions and libraries

**Core Stack**:
- Next.js 14.2+ (App Router, RSC)
- TypeScript 5.3+
- Tailwind CSS 3.4+
- React 18+

**UI Libraries**:
- shadcn/ui (component primitives)
- Framer Motion (animations)
- Lucide React (icons)
- next-themes (theme management)

**Content Processing**:
- @next/mdx or next-mdx-remote (MDX rendering)
- rehype-highlight (syntax highlighting)
- remark-gfm (GitHub Flavored Markdown)
- gray-matter (frontmatter parsing)

**3D/Animation** (choose one):
- React Three Fiber + Three.js (for 3D robot)
- OR Lottie React (for simpler animation)

---

### TA-003: Routing Strategy
**Priority**: P0
**Description**: Next.js 14 App Router file-based routing

**Route Structure**:
```
/                           → Landing page
/book/intro                 → Introduction
/book/module-1/week3        → Module 1, Week 3
/book/module-1/week4        → Module 1, Week 4
...
/book/hardware-setup        → Hardware guides
/book/assessments           → Assessment section
/book/references/glossary   → Glossary
```

**Route Conventions**:
- Use kebab-case for URLs (e.g., `/book/hardware-setup`)
- Module folders named: `module-1`, `module-2`, etc.
- Chapter files use descriptive names: `week3-nodes-topics.mdx`

---

### TA-004: MDX Content Schema
**Priority**: P0
**Description**: Frontmatter metadata structure for chapters

**Frontmatter Fields**:
```yaml
---
title: "Week 3: ROS 2 Nodes and Topics"
week: 3
module: 1
estimated_time: 4.5
difficulty: "Beginner"
prerequisites:
  - "Python basics"
  - "Linux terminal familiarity"
learning_objectives:
  - "Understand ROS 2 node architecture"
  - "Create publishers and subscribers"
  - "Debug node communication issues"
sidebar_label: "Week 3: Nodes & Topics"
---
```

**Field Specifications**:
- `title` (string, required): Full chapter title
- `week` (number, required): Week number (1-13)
- `module` (number, required): Module number (1-4)
- `estimated_time` (number, required): Hours to complete
- `difficulty` (enum, required): "Beginner" | "Intermediate" | "Advanced"
- `prerequisites` (array, optional): List of prerequisite topics
- `learning_obECTIVES` (array, required): List of learning goals
- `sidebar_label` (string, required): Shortened label for sidebar

---

## ✅ Acceptance Criteria

### AC-001: Landing Page
- [ ] Hero section displays with animation
- [ ] "Read Book" button is visually prominent
- [ ] 4 module cards render in responsive grid
- [ ] Features grid displays all 6 features
- [ ] Dark/Light mode toggle works smoothly
- [ ] All sections visible without horizontal scroll on mobile
- [ ] Lighthouse performance score ≥ 90

### AC-002: Book Navigation
- [ ] Sidebar shows complete content structure
- [ ] Clicking a module expands/collapses children
- [ ] Clicking a chapter navigates to that chapter
- [ ] Current chapter is highlighted
- [ ] Sidebar is collapsible on mobile
- [ ] Breadcrumb trail shows current location

### AC-003: Chapter Reading
- [ ] Chapter header displays all metadata correctly
- [ ] Markdown renders with proper styling
- [ ] Code blocks have syntax highlighting
- [ ] Copy button works on code blocks
- [ ] Images are responsive
- [ ] Table of Contents auto-generates from headings

### AC-004: Responsive Design
- [ ] Layout works on 320px width (small mobile)
- [ ] Layout works on 768px width (tablet)
- [ ] Layout works on 1024px width (desktop)
- [ ] Layout works on 1440px+ width (large desktop)
- [ ] Touch targets are min 44px on mobile
- [ ] No content cut off on any screen size

### AC-005: Performance
- [ ] Initial page load < 3 seconds
- [ ] Navigation between pages < 500ms
- [ ] No layout shift during load (CLS < 0.1)
- [ ] Images lazy load below fold
- [ ] No console errors or warnings

### AC-006: Accessibility
- [ ] Semantic HTML used throughout
- [ ] Keyboard navigation fully supported
- [ ] ARIA attributes for dynamic content
- [ ] Color contrast meets WCAG AA standards
- [ ] Focus indicators visible
- [ ] Screen reader compatibility tested

---

## 🗓️ Phase 1 Tasks (High-level)

- [ ] Set up Next.js project with Tailwind CSS and TypeScript
- [ ] Implement Landing Page (Hero, Modules, Features, Navbar, Theme toggle)
- [ ] Implement Book Layout (Sidebar, Chapter Header, Markdown Renderer)
- [ ] Add 3 placeholder MDX chapters
- [ ] Ensure responsive design across all breakpoints
- [ ] Integrate Framer Motion for animations
- [ ] Implement basic routing for book sections
- [ ] Integrate shadcn/ui components
- [ ] Performance and accessibility optimization
- [ ] Browser compatibility testing
- [ ] Final polish and bug fixes
- [ ] Prepare for Phase 2 (RAG chatbot)

---

## 🧪 Testing Strategy

**Manual Testing**:
- [ ] Test on Chrome, Firefox, Safari, Edge
- [ ] Test on iPhone (Safari)
- [ ] Test on Android (Chrome)
- [ ] Test on tablet (iPad)
- [ ] Test keyboard navigation
- [ ] Test with screen reader (NVDA/VoiceOver)

**Automated Testing**:
- [ ] Lighthouse CI in GitHub Actions
- [ ] Broken link checker
- [ ] Image optimization checker
- [ ] Bundle size monitoring

**User Testing**:
- [ ] Show to 2-3 peers for feedback
- [ ] Note pain points and confusion
- [ ] Iterate based on feedback

---

## 📖 Resources to Consult (via Context7)

**MUST research before implementing**:
- Next.js 14 App Router documentation
- Tailwind CSS v3 documentation
- shadcn/ui component documentation
- MDX documentation
- Framer Motion API reference
- Web accessibility guidelines (WCAG)
- React best practices
- TypeScript patterns

**Agent calls required**:
```
@context7-docs-analyst "Get Next.js 14 App Router file-based routing patterns"
@context7-docs-analyst "Get Tailwind CSS responsive design utilities"
@context7-docs-analyst "Get shadcn/ui Card component implementation"
@context7-docs-analyst "Get Framer Motion animation variants patterns"
@context7-docs-analyst "Get MDX rendering with syntax highlighting"
```

---

## 🎯 Definition of Done

**Phase 1 is complete when**:
- [ ] Landing page is live and visually stunning
- [ ] "Read Book" button navigates to book dashboard
- [ ] Book sidebar shows complete content structure
- [ ] At least 3 placeholder chapters render correctly
- [ ] Dark/Light mode works flawlessly
- [ ] All responsive breakpoints tested and polished
- [ ] Lighthouse scores meet targets (Performance ≥ 90, Accessibility ≥ 95)
- [ ] No console errors or warnings
- [ ] Code reviewed and approved
- [ ] Deployed to development environment
- [ ] RESOURCES.md updated with all work done

---

**Version**: 1.0
**Last Updated**: December 6, 2025
**Next Review**: After Phase 1 completion

## Response snapshot

It looks like the `sp.clarify` command cannot proceed because you are currently on the `master` branch. The `sp.clarify` command requires you to be on a feature branch (e.g., `001-book-master-plan`) to identify the feature's specification file.

Please switch to the `001-book-master-plan` branch to continue with the clarification process. You can do this by running:

`git checkout 001-book-master-plan`

Once you have switched branches, please run `/sp.clarify` again.

## Outcome

- ✅ Impact: User informed about incorrect branch for /sp.clarify.
- 🧪 Tests: N/A
- 📁 Files: N/A
- 🔁 Next prompts: User to switch branch and re-run /sp.clarify.
- 🧠 Reflection: Script failed due to incorrect branch, user needs to switch to feature branch.

## Evaluation notes (flywheel)

- Failure modes observed: `check-prerequisites.sh` returned error for not being on a feature branch.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
