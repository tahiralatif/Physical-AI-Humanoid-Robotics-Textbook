# Plan: Book Content (Docusaurus)

**Domain**: 002-book-content
**Spec**: specs/002-book-content/spec.md

## 1. Directory Structure
```
docusaurus/
├── docs/                   # The 4 Modules
│   ├── module-1-ros2/
│   ├── module-2-sim/
│   ├── module-3-isaac/
│   └── module-4-vla/
├── src/
│   ├── components/         # Custom React Embeds
│   └── css/                # Custom Styling
├── docusaurus.config.js    # Main Config
└── sidebars.js             # Navigation Logic
```

## 2. Implementation Steps
1.  **Initialize**: `npx create-docusaurus@latest docusaurus classic`
2.  **Configure**: Update `docusaurus.config.js` with project metadata.
3.  **Structure**: logic in `sidebars.js` to handle the 4-module split.
4.  **Content Generation**:
    *   Use Spec-Kit Plus prompts to generate MDX files.
    *   Ensure all images are in `static/img/`.
5.  **Subagent Review**: Run `book-architect` to validate structure.

## 3. Integration Prep
*   Ensure all H1/H2 tags are consistent for RAG chunking.
*   Add metadata frontmatter (`id`, `title`, `description`) for search optimization.
