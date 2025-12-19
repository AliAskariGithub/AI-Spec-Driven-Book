# Implementation Tasks: Docusaurus UI Upgrade

**Feature**: Docusaurus UI Upgrade
**Branch**: `006-docusaurus-ui`
**Spec**: [specs/006-docusaurus-ui/spec.md](../spec.md)
**Plan**: [specs/006-docusaurus-ui/plan.md](../plan.md)

## Implementation Strategy

**MVP Approach**: Implement User Story 1 (Landing Page Redesign) first to deliver core value early. Each user story is designed to be independently testable and incrementally deliverable.

**Parallel Execution Opportunities**: Tasks with [P] markers can be executed in parallel as they work on different files/components without dependencies.

**Testing Strategy**: Each user story includes validation tasks to ensure acceptance criteria are met.

---

## Phase 1: Setup & Project Initialization

### Goal
Initialize the project structure and ensure all necessary dependencies and assets are in place for the UI upgrade.

- [X] T001 Create necessary directory structure for components in `fullstack/frontend-book/src/components/`
- [X] T002 Verify Docusaurus v3 installation and dependencies in `fullstack/frontend-book/package.json`
- [X] T003 Verify font assets exist in `fullstack/frontend-book/static/fonts/` (Archivo, General Sans, SourceSerif4)
- [X] T004 Verify existing Docusaurus configuration files are accessible (`docusaurus.config.js`, `sidebars.js`)

---

## Phase 2: Foundational & Blocking Prerequisites

### Goal
Establish the foundational elements required for all user stories: theme system, typography system, and CSS customizations.

- [X] T005 [P] Create custom CSS file `fullstack/frontend-book/src/css/custom.css` with indigo primary color variables
- [X] T006 [P] Create font CSS file `fullstack/frontend-book/src/css/fonts.css` with @font-face declarations for Archivo, General Sans, and SourceSerif4
- [X] T007 [P] Update `docusaurus.config.js` to set dark mode as default and configure theme properties
- [X] T008 [P] Create `fullstack/frontend-book/src/components/ThemeSystem/index.js` with theme context provider
- [X] T009 [P] Create `fullstack/frontend-book/src/components/TypographySystem/index.js` with typography wrapper component
- [X] T010 [P] Create `fullstack/frontend-book/src/components/ThemeToggleButton/index.js` for theme switching functionality

---

## Phase 3: User Story 1 - Landing Page Redesign (P1)

### Goal
Redesign the landing page with four distinct sections: Intro with two buttons, Three feature cards with hover animations, What you will learn, and Featured chapters.

**Independent Test Criteria**: Landing page displays all four required sections with proper typography, color scheme (#6C3BAA), hover animations, and responsive design.

- [X] T011 [P] [US1] Create `fullstack/frontend-book/src/components/FeatureCard/index.js` component with hover animations
- [X] T012 [P] [US1] Create `fullstack/frontend-book/src/components/FeatureCard/styles.module.css` with hover effects
- [X] T013 [P] [US1] Create `fullstack/frontend-book/src/components/ResponsiveCardGrid/index.js` for responsive card layout
- [X] T014 [US1] Create `fullstack/frontend-book/src/components/LearningOutcomes/index.js` component for "What you will learn" section
- [X] T015 [US1] Create `fullstack/frontend-book/src/components/FeaturedChapters/index.js` component for featured chapters section
- [X] T016 [US1] Create `fullstack/frontend-book/src/components/AccessibleButton/index.js` component with keyboard accessibility
- [X] T017 [US1] Update `fullstack/frontend-book/src/pages/index.js` to implement new landing page layout with all required sections
- [X] T018 [US1] Update `fullstack/frontend-book/src/pages/index.module.css` with new landing page styles
- [X] T019 [US1] Implement typography requirements for landing page (Archivo headings, General Sans body text)
- [X] T020 [US1] Validate landing page meets accessibility requirements (keyboard navigation, ARIA labels)
- [X] T021 [US1] Test responsive design across mobile, tablet, and desktop
- [X] T022 [US1] Verify all four sections appear with proper styling and interactive elements

---

## Phase 4: User Story 2 - Navigation System Enhancement (P2)

### Goal
Enhance the navigation system with consistent header and footer navigation that supports dark/light mode.

**Independent Test Criteria**: Navbar with text logo, textbook link, search bar, and theme toggle functions correctly across all pages; footer with organized sections is consistently displayed.

- [X] T023 [P] [US2] Create `fullstack/frontend-book/src/components/NavigationBar/index.js` with text logo, textbook link, search bar, and theme toggle
- [X] T024 [P] [US2] Create `fullstack/frontend-book/src/components/NavigationBar/styles.module.css` with responsive navigation styles
- [X] T025 [US2] Create `fullstack/frontend-book/src/components/FooterSections/index.js` with Learn, Community, Resources, About sections
- [X] T026 [US2] Create `fullstack/frontend-book/src/components/FooterSections/styles.module.css` with organized footer layout
- [X] T027 [US2] Update `docusaurus.config.js` to integrate new navigation components
- [X] T028 [US2] Implement theme toggle functionality that works across all pages
- [X] T029 [US2] Test navigation accessibility (keyboard navigation, screen reader compatibility)
- [X] T030 [US2] Verify footer sections are displayed consistently across all pages
- [X] T031 [US2] Validate search bar functionality integration
- [X] T032 [US2] Test navigation responsiveness across device sizes

---

## Phase 5: User Story 3 - Textbook Content Layout Upgrade (P3)

### Goal
Upgrade textbook content layout with improved typography following specified font standards and enhanced hover states.

**Independent Test Criteria**: Textbook pages display headings with Archivo font and body text with SourceSerif4 font with proper contrast; hover states are clearly visible for sidebar navigation.

- [X] T033 [P] [US3] Create `fullstack/frontend-book/src/components/TextbookPageWrapper/index.js` for textbook-specific typography
- [X] T034 [P] [US3] Create `fullstack/frontend-book/src/components/SidebarEnhancement/index.js` with improved hover states
- [X] T035 [US3] Update textbook page templates to use SourceSerif4 for body text and Archivo for headings
- [X] T036 [US3] Enhance sidebar navigation with improved hover states as specified in requirements
- [X] T037 [US3] Implement typography requirements for textbook pages (Archivo headings, SourceSerif4 body text)
- [X] T038 [US3] Validate contrast ratios meet WCAG 2.1 AA standards for textbook content
- [X] T039 [US3] Test theme switching functionality on textbook pages
- [X] T040 [US3] Verify all existing content remains accessible and unchanged (no content loss)
- [X] T041 [US3] Test hover states on sidebar navigation elements
- [X] T042 [US3] Validate responsive design for textbook content pages

---

## Phase 6: Polish & Cross-Cutting Concerns

### Goal
Final validation, optimization, and polish to ensure all requirements are met and the implementation is production-ready.

- [X] T043 Validate all functional requirements (FR-001 through FR-013) are implemented
- [X] T044 Test all success criteria (SC-001 through SC-008) are satisfied
- [X] T045 Perform accessibility audit to ensure WCAG 2.1 AA compliance
- [X] T046 Optimize font loading with `font-display: swap` for performance
- [X] T047 Test edge cases: custom browser font settings, large/small screens, JavaScript disabled
- [X] T048 Validate responsive design across all device sizes
- [X] T049 Perform cross-browser testing (Chrome, Firefox, Safari, Edge)
- [X] T050 Verify no existing content or markdown structure was lost
- [X] T051 Update documentation if needed for the new components
- [X] T052 Run performance tests to ensure <3s page load time
- [X] T053 Final validation of indigo primary color (#6C3BAA) implementation throughout UI
- [X] T054 Verify all interactive elements are keyboard accessible (FR-012)

---

## Dependencies & Execution Order

### User Story Dependencies
- **US1 (P1)**: No dependencies - can be implemented first
- **US2 (P2)**: Depends on ThemeSystem from Phase 2
- **US3 (P3)**: Depends on TypographySystem and ThemeSystem from Phase 2

### Parallel Execution Examples
- Tasks T011-T013 can run in parallel as they create different components
- Tasks T023-T024 can run in parallel as they create navigation components
- Tasks T033-T034 can run in parallel as they create textbook-specific components

### Critical Path
1. Phase 1 & 2 must complete before any user stories
2. US1 can be completed independently and provides MVP functionality
3. US2 and US3 can be developed after US1, but are not dependent on each other

---

## MVP Scope (Recommended for Initial Delivery)

Focus on User Story 1 (Landing Page Redesign) which delivers the most visible value to users:
- Tasks T001-T022 (setup, foundation, and landing page implementation)
- Provides immediate visual improvement and demonstrates the new UI direction
- Includes all core functionality: typography, theme system, and interactive elements