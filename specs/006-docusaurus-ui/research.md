# Research: Docusaurus UI Upgrade

## Decision: Docusaurus Theme Customization Approach
**Rationale**: Using Docusaurus's theme customization capabilities is the most appropriate approach for this UI upgrade, as it allows for comprehensive UI changes while preserving the existing content structure and Docusaurus functionality. This approach follows Docusaurus best practices and ensures maintainability.

**Alternatives considered**:
- Complete rebuild with custom React app: Would lose Docusaurus benefits and require reimplementation of existing functionality
- Third-party Docusaurus themes: May not provide the specific customization required for the indigo color scheme and typography requirements

## Decision: Typography Implementation
**Rationale**: Implement typography through CSS custom properties and component-level styling to ensure consistent application of Archivo/General Sans for landing pages and Archivo/SourceSerif4 for textbook content. This approach provides the flexibility needed for different font applications per page type while maintaining performance.

**Alternatives considered**:
- Inline styles: Would be difficult to maintain and inconsistent
- External CSS without Docusaurus integration: May conflict with existing styles

## Decision: Dark/Light Mode Implementation
**Rationale**: Use Docusaurus's built-in dark mode capabilities with custom CSS variables for the indigo primary color (#6C3BAA). This approach leverages Docusaurus's existing theme switching functionality while allowing for custom color schemes.

**Alternatives considered**:
- Custom theme switching implementation: Would duplicate existing functionality
- Third-party theme libraries: May not integrate well with Docusaurus

## Decision: Component Architecture
**Rationale**: Create custom React components for interactive elements (feature cards, buttons, navigation elements) following Docusaurus component conventions. This ensures proper integration with the Docusaurus ecosystem while providing the required interactivity and hover states.

**Alternatives considered**:
- Pure CSS solutions: Would not provide required interactivity
- External UI libraries: May conflict with Docusaurus styling and increase bundle size

## Technology Research: Docusaurus Best Practices

### Theme Customization
- Docusaurus allows for comprehensive theme customization through CSS modules and component swizzling
- Custom CSS can be added via the `src/css/custom.css` file referenced in `docusaurus.config.js`
- Components can be customized by creating files in `src/components/` with the same name as theme components

### Typography in Docusaurus
- Custom fonts can be loaded via CSS `@font-face` declarations
- Typography can be customized using CSS variables
- Different typography for different page types requires conditional styling or custom components

### Responsive Design
- Docusaurus uses CSS Grid and Flexbox for responsive layouts
- Media queries can be used to ensure responsive behavior across devices
- Docusaurus provides built-in responsive utilities

### Accessibility (WCAG 2.1 AA)
- Docusaurus follows accessibility best practices by default
- Custom components must implement proper ARIA attributes
- Color contrast ratios must meet 4.5:1 for normal text, 3:1 for large text
- Keyboard navigation must be preserved for all interactive elements

### Performance Considerations
- Minimize custom CSS and JavaScript to maintain performance
- Optimize font loading with `font-display: swap`
- Use CSS containment where appropriate for performance

## Dependencies and Setup Requirements

### Font Loading
- Archivo font for headings (available via Google Fonts or self-hosted)
- General Sans for landing page body text (self-hosted)
- SourceSerif4 for textbook body text (self-hosted in existing fonts folder)

### Development Dependencies
- Docusaurus v3 with required plugins
- React and React DOM
- CSS modules support
- Optional: TypeScript support if needed

## Implementation Strategy

### Phase 1: Foundation
1. Set up custom CSS with color variables and typography definitions
2. Implement dark/light mode with indigo primary color
3. Add font loading for all required typefaces

### Phase 2: Components
1. Create custom components for feature cards with hover animations
2. Implement interactive buttons with proper hover states
3. Update navigation components to match design requirements

### Phase 3: Integration
1. Update landing page to use new components and styling
2. Apply typography changes to textbook pages
3. Ensure responsive design works across all devices
4. Verify accessibility compliance

## Known Challenges and Solutions

### Challenge: Maintaining Existing Content Structure
**Solution**: Use Docusaurus theme customization instead of content modification. Focus on styling and component changes rather than content changes.

### Challenge: Different Typography for Different Page Types
**Solution**: Use CSS classes applied conditionally based on page type, or create different component variants for landing vs. textbook pages.

### Challenge: Responsive Hover Effects on Mobile
**Solution**: Implement hover effects that degrade gracefully on touch devices, using focus states for accessibility.

## Edge Cases Addressed

### Custom Browser Font Settings
- Implement proper font fallback chains
- Use CSS `font-display: swap` for better loading behavior

### Large Screen Resolutions
- Use max-width constraints to maintain readability
- Implement responsive grid layouts that adapt to screen size

### JavaScript Disabled
- Ensure core navigation remains functional
- Provide graceful degradation for interactive elements

### Text Length Variations in Different Languages
- Use flexible layouts that adapt to text length
- Implement text truncation or wrapping as needed