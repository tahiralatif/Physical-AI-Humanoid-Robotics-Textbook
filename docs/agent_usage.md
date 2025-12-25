# Agent Usage Guide: Physical AI & Humanoid Robotics Textbook

This project utilizes custom **Claude Code Subagents** to maintain high standards and accelerate development. These agents are located in `.claude/agents/`.

## 🤖 Available Agents

### 1. Book Architect (`book-architect.md`)
- **Role**: Editor-in-Chief.
- **When to use**: 
    - Structuring new chapters.
    - Reviewing technical accuracy of ROS 2 or NVIDIA Isaac content.
    - Ensuring pedagogical flow.
- **Commands**: `/sp.phr` with label `book-architect`.

### 2. Quiz Generator (`quiz-generator.md`)
- **Role**: Assessment Specialist.
- **When to use**:
    - Generating weekly MCQs.
    - Designing capstone project rubrics.
    - Creating interactive lab challenges.

### 3. Project Manager (`project-manager.md`)
- **Role**: Timeline & Task Overseer.
- **When to use**:
    - Updating `tasks.md`.
    - Identifying blockers.
    - Providing project health reports.

### 4. Tech Lead (`tech-lead.md`)
- **Role**: Code Integrity & Architecture.
- **When to use**:
    - Reviewing FastAPI service logic.
    - Auditing Next.js frontend components.
    - Ensuring security best practices.

## 🛠️ How to Invoke
When working in the Claude CLI, you can reference these agents to context-shift your assistant:

"As the **Book Architect**, please review `docusaurus/docs/module-1-ros2/week-3-fundamentals.md` for technical depth."

## 📈 Agent Workflows
1. **Planning**: Use `Project Manager` to verify `tasks.md` before starting a feature.
2. **Implementation**: Use `Tech Lead` to review API contracts in `fastapi/`.
3. **Content**: Use `Book Architect` to generate outlines and `Quiz Generator` to add knowledge checks.
