# Environment Configuration Guide

## Setup Instructions

Since `.env` files are gitignored, you need to create them manually based on the templates below:

### 1. Docusaurus (.env)
Create `docusaurus/.env`:
```
DOCUSAURUS_URL=http://localhost:3000
```

### 2. Next.js (.env.local)
Create `nextjs/.env.local`:
```
NEXT_PUBLIC_API_URL=http://localhost:8000
NEXTAUTH_SECRET=your-secret-key-here-change-in-production
DATABASE_URL=postgresql://user:password@host:5432/physical_ai_textbook
OPENAI_API_KEY=sk-your-openai-api-key-here
```

### 3. FastAPI (.env)
Create `fastapi/.env`:
```
DATABASE_URL=postgresql://user:password@host:5432/physical_ai_textbook
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key-here
OPENAI_API_KEY=sk-your-openai-api-key-here
ENV=development
DEBUG=true
```

## Cloud Services Required

### Neon Serverless Postgres (T008)
1. Sign up at https://neon.tech
2. Create new project
3. Copy connection string to `DATABASE_URL`

### Qdrant Cloud (T009)
1. Sign up at https://cloud.qdrant.io
2. Create free tier cluster
3. Copy URL and API key to env files

### OpenAI
1. Get API key from https://platform.openai.com
2. Add to both Next.js and FastAPI .env files
