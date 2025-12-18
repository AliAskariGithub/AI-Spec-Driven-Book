---
id: 1
title: Create Constitution
stage: constitution
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: unknown
command: /sp.constitution
labels: [constitution,spec-first,ai-assisted]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```
/sp.constitution

Project: AI-Spec–Driven Book with Embedded RAG Chatbot

Core principles:
- Spec-first development using Spec-Kit Plus
- AI-assisted authoring via Claude Code
- Technical accuracy from official documentation
- Clear, developer-focused writing
- Fully reproducible workflows

Key standards:
- All technical claims traceable to official docs or reputable sources
- Clean, modular code with env-based configuration
- Consistent terminology and progressive learning structure
- No hard-coded secrets or paid-only dependencies

Book requirements:
- Built with Docusaurus
- Deployed to GitHub Pages
- Includes tutorials, architecture explanations, and code examples
- Documents Spec-Kit Plus + Claude Code workflows

RAG chatbot requirements:
- Embedded in the book UI
- Backend: FastAPI + OpenAI Agents/ChatKit
- Storage: Neon Serverless Postgres + Qdrant Cloud (free tier)
- Answers:
  - Book-wide questions
  - Questions limited to user-selected text
- Retrieval required before generation; no hallucinations

Constraints:
- Public, reproducible repository
- Fresh-machine setup documented
- `.env.example` provided
- Specs map directly to implementation

Success criteria:
- Book live on GitHub Pages
- Chatbot works reliably in production
- RAG answers are accurate and context-grounded
- Project rebuildable end-to-end from specs
```

## Response snapshot

Created the project constitution file with all specified principles and requirements. The constitution establishes core principles for spec-first development, AI-assisted authoring, technical accuracy, reproducible workflows, clean architecture, and modular code. It defines technical standards for the technology stack and quality requirements, plus development workflow processes and governance rules.

## Outcome

- ✅ Impact: Created initial project constitution at .specify/memory/constitution.md with 6 core principles and governance structure
- 🧪 Tests: none
- 📁 Files: 1 file created (.specify/memory/constitution.md)
- 🔁 Next prompts: Continue with spec creation using /sp.specify
- 🧠 Reflection: Constitution successfully captures all specified project principles and requirements

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin spec creation for the book and RAG chatbot