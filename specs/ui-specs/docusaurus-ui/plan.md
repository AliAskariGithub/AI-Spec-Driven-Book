# Implementation Plan: Docusaurus UI Upgrade

**Branch**: `006-docusaurus-ui` | **Date**: 2025-12-19 | **Spec**: specs/006-docusaurus-ui/spec.md
**Input**: Feature specification from `/specs/006-docusaurus-ui/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a comprehensive UI upgrade for the Docusaurus-based textbook website, focusing on modern design elements, improved typography (Archivo/General Sans for landing, Archivo/SourceSerif4 for content), dark/light mode with indigo primary color (#6C3BAA), enhanced navigation, and interactive components with proper accessibility and responsive design.

## Technical Context

**Language/Version**: JavaScript/TypeScript, Node.js v18+ (as required by Docusaurus v3)
**Primary Dependencies**: Docusaurus v3, React 18, Node.js, npm/yarn
**Storage**: N/A (static site generation)
**Testing**: Jest for unit tests, Cypress for E2E tests (NEEDS CLARIFICATION based on project setup)
**Target Platform**: Web (static site deployed to GitHub Pages)
**Project Type**: Web frontend (Docusaurus static site)
**Performance Goals**: <3s page load time, <100ms interaction response, Core Web Vitals compliant
**Constraints**: Must maintain all existing content and markdown structure, WCAG 2.1 AA compliance, responsive across all device sizes
**Scale/Scope**: Single textbook website with multiple pages and sections

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the constitution file, the following gates must be satisfied:
1. ✅ Spec-First Development: Specification already completed and approved in spec.md
2. ✅ AI-Assisted Development: All work has been done with Claude Code assistance
3. ✅ Technical Accuracy and Documentation: Implementation follows official Docusaurus documentation
4. ✅ Reproducible Workflows: Docusaurus provides standard setup and deployment workflows, documented in quickstart.md
5. ✅ Clean Architecture and Security: No hard-coded secrets; using environment variables where needed
6. ✅ Modular Code with Environment-Based Configuration: Following Docusaurus component architecture with modular components defined in contracts/

## Project Structure

### Documentation (this feature)

```text
specs/006-docusaurus-ui/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
fullstack/frontend-book/
├── src/
│   ├── components/      # Custom React components (buttons, cards, etc.)
│   ├── pages/          # Landing page and other custom pages
│   ├── css/            # Custom CSS and theme files
│   └── utils/          # Utility functions
├── static/             # Static assets (fonts, images, etc.)
│   └── fonts/          # Font files (Archivo, General Sans, SourceSerif4)
├── docs/               # Documentation content (preserved)
├── docusaurus.config.js # Docusaurus configuration
├── sidebars.js         # Navigation sidebar configuration
└── package.json        # Project dependencies
```

**Structure Decision**: Single Docusaurus project structure, as this is a frontend-only UI upgrade for an existing static site. The implementation will leverage Docusaurus's component system and theme customization capabilities to implement the required UI changes without affecting the underlying content structure.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | N/A | N/A (All constitution checks passed) |
