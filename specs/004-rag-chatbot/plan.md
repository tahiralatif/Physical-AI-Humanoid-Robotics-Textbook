# Plan: RAG Chatbot (FastAPI)

**Domain**: 004-rag-chatbot
**Spec**: specs/004-rag-chatbot/spec.md

## 1. Directory Structure
```
fastapi/
├── app/
│   ├── api/                # Endpoints (v1)
│   ├── core/               # Config, Security
│   ├── services/           # Chat, Qdrant, OpenAI Logic
│   └── models/             # Pydantic Schemas
├── scripts/
│   └── ingest.py           # ETL Pipeline
└── requirements.txt
```

## 2. Implementation Steps
1.  **Initialize**: Setup Python environment and FastAPI skeleton.
2.  **Database**: Connect to Qdrant and Neon Postgres.
3.  **Ingestion**: Implement `ingest.py` to parse `../docusaurus/docs`.
4.  **RAG Service**: Build the Search & Answer logic.
5.  **API Layer**: Expose `POST /chat`, `POST /personalize`, `POST /translate`.
6.  **Optimization**: Implement Caching (Redis or simple In-Memory/DB) for LLM responses.

## 3. Subagent
*   Execute `rag-architect` to build the Service Layer.
