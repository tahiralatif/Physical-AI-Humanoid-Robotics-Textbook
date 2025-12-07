# Deployment Guide

## Quick Deployment Options

### Option 1: GitHub Pages (Recommended for Docusaurus)

1. **Enable GitHub Pages in your repo:**
   - Go to Settings → Pages
   - Source: GitHub Actions

2. **Update `docusaurus/docusaurus.config.ts`:**
   ```typescript
   url: 'https://YOUR_USERNAME.github.io',
   baseUrl: '/Physical-AI-Humanoid-Robotics-Textbook/',
   organizationName: 'YOUR_USERNAME',
   projectName: 'Physical-AI-Humanoid-Robotics-Textbook',
   ```

3. **Push to GitHub:**
   ```bash
   git add .
   git commit -m "Add textbook content"
   git push origin main
   ```

4. **GitHub Actions will automatically deploy!**
   - Visit: `https://YOUR_USERNAME.github.io/Physical-AI-Humanoid-Robotics-Textbook/`

### Option 2: Vercel (Recommended for Full Stack)

1. **Push to GitHub**

2. **Import to Vercel:**
   - Visit https://vercel.com/new
   - Select your repository
   - Configure build settings:
     - **Docusaurus Project:**
       - Root directory: `docusaurus`
       - Build command: `npm run build`
       - Output directory: `build`
     
     - **Next.js Project:**
       - Root directory: `nextjs`
       - Build command: `npm run build`
       - Output directory: `.next`

3. **Set Environment Variables in Vercel:**
   ```
   DATABASE_URL=your_neon_postgres_url
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_key
   OPENAI_API_KEY=your_openai_key
   NEXTAUTH_SECRET=generate_random_secret
   ```

4. **Deploy FastAPI:**
   - Create `vercel.json` in root:
   ```json
   {
     "builds": [
       {"src": "fastapi/main.py", "use": "@vercel/python"}
     ],
     "routes": [
       {"src": "/api/(.*)", "dest": "fastapi/main.py"}
     ]
   }
   ```

### Option 3: Manual Deployment

#### Docusaurus
```bash
cd docusaurus
npm run build
# Upload build/ folder to any static hosting (Netlify, Cloudflare Pages, etc.)
```

#### FastAPI
```bash
cd fastapi
# Deploy to Railway, Render, or Fly.io
```

## What You Need for Clone Setup

Anyone cloning your repo will need:

### 1. Software Requirements
- Node.js 18+
- Python 3.10+
- Git
- UV package manager (for FastAPI)

### 2. Cloud Services (Free Tiers)
- **Neon Postgres** (database) - https://neon.tech
- **Qdrant Cloud** (vector search) - https://cloud.qdrant.io
- **OpenAI API Key** - https://platform.openai.com

### 3. Setup Steps
```bash
# Clone
git clone https://github.com/YOUR_USERNAME/Physical-AI-Humanoid-Robotics-Textbook.git
cd Physical-AI-Humanoid-Robotics-Textbook

# Install Docusaurus
cd docusaurus
npm install
npm start  # View at http://localhost:3000

# Install Next.js (in new terminal)
cd nextjs
npm install
npm run dev

# Install FastAPI (in new terminal)
cd fastapi
uv sync
uv run uvicorn main:app --reload
```

### 4. Environment Setup
- Create `.env` files (see ENV_SETUP.md)
- Run database migrations
- Configure API keys

## Estimated Time for Clone Setup
- **Textbook only (Docusaurus)**: 5-10 minutes
- **Full stack (all 3 apps)**: 30-45 minutes
- **With cloud services**: +15 minutes

## Next Steps After Deployment

Once deployed, you can:
1. ✅ Submit the Docusaurus link for hackathon (40 points)
2. Implement RAG Chatbot (+60 points) 
3. Add bonus features (+200 points)

Ready to deploy? Let me know which option you prefer!
