<!--
Sync Impact Report:
- Version change: N/A → 1.0.0 (initial creation)
- Modified principles: N/A (new file)
- Added sections: All sections (new file)
- Removed sections: None
- Templates requiring updates: ✅ No templates affected (new file)
- Follow-up TODOs: None
-->

# AI-Spec–Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Spec-First Development (NON-NEGOTIABLE)
All development begins with comprehensive specifications using Spec-Kit Plus; Specifications must be complete and approved before implementation begins; Every feature, requirement, and constraint must be documented in specs before coding starts.

### II. AI-Assisted Development
Leverage Claude Code for all development tasks; All code changes must be reviewed and validated by AI assistance; Maintain human oversight and approval for all AI-generated code before integration.

### III. Technical Accuracy and Documentation
All technical claims must be traceable to official documentation or reputable sources; Code examples must be tested and verified for accuracy; Comprehensive documentation required for all features and workflows.

### IV. Reproducible Workflows
All development workflows must be fully reproducible on fresh machines; Setup processes must be documented with clear, step-by-step instructions; Dependency management must be version-controlled and deterministic.

### V. Clean Architecture and Security
No hard-coded secrets or credentials allowed; Use environment variables for all configuration; Implement clean separation of concerns between book content and RAG chatbot functionality.

### VI. Modular Code with Environment-Based Configuration

All services must be independently configurable via environment variables; Code must be modular with clear interfaces between components; Configuration must be documented in .env.example files.

## Technical Standards

### Technology Stack Requirements
- Book platform: Docusaurus for static site generation
- Deployment: GitHub Pages for public hosting
- Backend: FastAPI for RAG chatbot API
- Database: Neon Serverless Postgres for metadata
- Vector storage: Qdrant Cloud (free tier) for embeddings
- Frontend: Embedded chatbot UI in Docusaurus pages

### Quality Standards
- All code must follow consistent terminology and progressive learning structure
- Technical claims must be verifiable through official documentation
- Code examples must be runnable and produce documented outputs
- All dependencies must be free-tier compatible or open-source

## Development Workflow

### Implementation Process
- Specs map directly to implementation without deviation
- All changes must maintain backward compatibility where applicable
- Code reviews must verify compliance with constitution principles
- Testing required for all new features and bug fixes

### Deployment and Release
- Automated deployment to GitHub Pages from main branch
- RAG chatbot backend deployed with documented infrastructure as code
- Versioning follows semantic versioning (MAJOR.MINOR.PATCH)
- Rollback procedures documented for all deployments

## Governance

This constitution governs all development activities for the AI-Spec–Driven Book with Embedded RAG Chatbot project. All team members must comply with these principles. Amendments require formal documentation and approval process. All pull requests and code reviews must verify constitution compliance. Project success is measured by meeting the defined success criteria: book live on GitHub Pages, reliable chatbot operation, accurate RAG answers, and end-to-end reproducibility.

**Version**: 1.0.0 | **Ratified**: 2025-12-17 | **Last Amended**: 2025-12-1