# Specification: Book Content (Docusaurus)

**Domain**: 002-book-content
**Focus**: Static Documentation Site, Educational Content, Sidebar Structure.

## 1. Goal
Build the core educational content for "Physical AI & Humanoid Robotics" using **Docusaurus**. This component is strictly for hosting the textbook chapters, images, and static resources. It serves as the data source for the RAG chatbot and the content provider for the Next.js UI integration.

## 2. Requirements

### Content Structure (Spec-Kit Plus Driven)
The book must follow the 13-week curriculum:
*   **Module 1**: ROS 2 (Weeks 3-5)
*   **Module 2**: Gazebo & Unity (Weeks 6-7)
*   **Module 3**: NVIDIA Isaac (Weeks 8-10)
*   **Module 4**: VLA & Capstone (Weeks 11-13)

### Technical Specs
*   **Framework**: Docusaurus 3.x
*   **Theme**: Standard Docusaurus Preset, customized for "Panaversity" branding.
*   **Markdown Features**:
    *   Use Admonitions (Note, Tip, Warning).
    *   Use Code Blocks with Syntax Highlighting (Python, C++, XML/URDF, Bash).
    *   Use Tabs for Multi-OS instructions (Linux/Windows).
*   **Integration Points**:
    *   Must expose content in a format parsable by the RAG Ingestion script (clean Markdown).
    *   Must allow embedding of React Components (from Next.js or local) for "Personalize" buttons.

## 3. Subagent Requirement
*   Use `book-architect` agent from `.claude/agents/` to generate chapter outlines and review technical accuracy.
