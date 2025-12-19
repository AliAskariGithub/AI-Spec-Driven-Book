# Data Model: Docusaurus UI Upgrade

## Entity: Landing Page Layout
- **Name**: LandingPageLayout
- **Fields**:
  - sections: Array<Section>
  - heroContent: HeroContent
  - featureCards: Array<FeatureCard>
  - learningOutcomes: Array<string>
  - featuredChapters: Array<ChapterReference>
- **Relationships**:
  - Contains multiple Section entities
  - Contains multiple FeatureCard entities
- **Validation rules**:
  - Must include exactly 4 sections as specified in requirements (FR-001)
  - All interactive elements must be accessible (FR-0012)

## Entity: Section
- **Name**: Section
- **Fields**:
  - id: string
  - title: string
  - content: string | ReactNode
  - type: "hero" | "features" | "learning-outcomes" | "featured-chapters"
  - styles: CSSProperties
- **Relationships**:
  - Belongs to LandingPageLayout
- **Validation rules**:
  - Type must be one of the 4 required section types
  - Title and content must not be empty

## Entity: FeatureCard
- **Name**: FeatureCard
- **Fields**:
  - id: string
  - title: string
  - description: string
  - icon: string | ReactComponent
  - hoverState: HoverState
  - link: string
  - animation: AnimationType
- **Relationships**:
  - Belongs to LandingPageLayout
- **Validation rules**:
  - Must support hover animations (FR-010)
  - Must be interactive and readable (from scope)
  - Must be keyboard accessible (FR-012)

## Entity: HoverState
- **Name**: HoverState
- **Fields**:
  - transform: string
  - boxShadow: string
  - transition: string
  - isActive: boolean
- **Relationships**:
  - Belongs to FeatureCard
- **State transitions**:
  - inactive → active (on hover/focus)
  - active → inactive (on mouse leave/focus out)

## Entity: HeroContent
- **Name**: HeroContent
- **Fields**:
  - title: string
  - subtitle: string
  - buttons: Array<Button>
  - backgroundImage: string (optional)
- **Relationships**:
  - Belongs to LandingPageLayout
- **Validation rules**:
  - Must include 2 buttons as specified in scope
  - Title and subtitle must use proper typography (FR-002)

## Entity: Button
- **Name**: Button
- **Fields**:
  - id: string
  - text: string
  - link: string
  - variant: "primary" | "secondary" | "outline"
  - hoverState: HoverState
  - accessibilityLabel: string
- **Relationships**:
  - Belongs to HeroContent
- **Validation rules**:
  - Must be keyboard accessible (FR-012)
  - Must have proper ARIA labels for accessibility

## Entity: ChapterReference
- **Name**: ChapterReference
- **Fields**:
  - id: string
  - title: string
  - description: string
  - path: string
  - level: "beginner" | "intermediate" | "advanced"
- **Relationships**:
  - Belongs to LandingPageLayout (featuredChapters)
- **Validation rules**:
  - Path must point to valid documentation page

## Entity: TypographySystem
- **Name**: TypographySystem
- **Fields**:
  - pageType: "landing" | "textbook"
  - headingFont: FontSpec
  - bodyFont: FontSpec
  - fallbackFonts: Array<string>
- **Relationships**:
  - Applied to various UI components based on page type
- **Validation rules**:
  - Landing pages must use Archivo/General Sans (FR-002)
  - Textbook pages must use Archivo/SourceSerif4 (FR-003)

## Entity: FontSpec
- **Name**: FontSpec
- **Fields**:
  - fontFamily: string
  - fontWeight: number | string
  - fontStyle: string
  - fontDisplay: "swap" | "block" | "fallback" | "optional"
- **Relationships**:
  - Belongs to TypographySystem
- **Validation rules**:
  - Must have valid CSS font properties
  - Must include fallback fonts

## Entity: ThemeSystem
- **Name**: ThemeSystem
- **Fields**:
  - mode: "dark" | "light"
  - primaryColor: string (hex color)
  - secondaryColors: Array<string>
  - contrastRatios: ContrastRatios
  - toggleFunctionality: boolean
- **Relationships**:
  - Applied globally to all UI components
- **Validation rules**:
  - Dark mode must be default (FR-004)
  - Primary color must be #6C3BAA (FR-005)
  - Must meet WCAG 2.1 AA contrast requirements (FR-007)

## Entity: ContrastRatios
- **Name**: ContrastRatios
- **Fields**:
  - normalText: number (min 4.5)
  - largeText: number (min 3.0)
  - graphicalObjects: number (min 3.0)
- **Relationships**:
  - Belongs to ThemeSystem
- **Validation rules**:
  - All ratios must meet WCAG 2.1 AA minimums

## Entity: NavigationSystem
- **Name**: NavigationSystem
- **Fields**:
  - navbar: NavbarComponent
  - footer: FooterComponent
  - sidebar: SidebarComponent
  - searchEnabled: boolean
  - themeToggleEnabled: boolean
- **Relationships**:
  - Contains multiple navigation components
- **Validation rules**:
  - Must include text logo, textbook link, search bar, theme toggle (FR-008)
  - Footer must have Learn, Community, Resources, About sections (FR-009)
  - All interactive elements must be keyboard accessible (FR-012)

## Entity: NavbarComponent
- **Name**: NavbarComponent
- **Fields**:
  - logo: string | ReactComponent
  - textLogo: string
  - links: Array<LinkItem>
  - searchEnabled: boolean
  - themeToggle: boolean
- **Relationships**:
  - Belongs to NavigationSystem
- **Validation rules**:
  - Must include textbook link as specified (FR-008)

## Entity: LinkItem
- **Name**: LinkItem
- **Fields**:
  - id: string
  - text: string
  - path: string
  - section: "learn" | "community" | "resources" | "about"
- **Relationships**:
  - Belongs to NavbarComponent or FooterComponent
- **Validation rules**:
  - Footer links must be organized in the 4 required sections (FR-009)