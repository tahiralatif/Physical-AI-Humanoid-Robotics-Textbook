# Specification: RAG Chatbot (FastAPI)

**Domain**: 004-rag-chatbot
**Focus**: AI Backend, Vector Search, Personalization Service.

## 1. Goal
Provide the "Brain" for the platform. This backend handles all AI logic: Ingesting the book content, answering student questions via RAG, personalizing content based on user profiles, and handling translations.

## 2. Requirements

### Core RAG Pipeline
*   **Ingestion**: Script to recursively read Docusaurus Markdown files, chunk them by headers, and generate embeddings.
*   **Storage**: Qdrant Cloud (Vector DB).
*   **Retrieval**: Hybrid search (Dense Vector + Keyword) to find relevant book sections.
*   **Generation**: OpenAI GPT-4o-mini (or similar) to synthesize answers with citations.

### Bonus APIs
*   **Personalization API**: Endpoint that accepts `chapter_content` + `user_context` and returns a rewritten version.
*   **Translation API**: Endpoint that translates content to Urdu while preserving code blocks.
*   **Auth Middleware**: Verify Better-Auth JWT tokens.

## 3. Subagent Requirement
*   Use `rag-architect` agent from `.claude/agents/` to design the API schema and ingestion pipeline.
