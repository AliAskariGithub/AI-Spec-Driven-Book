# Data Model: Module 2 - Digital Twin Educational Content

## Entity: Module
**Description**: Container for related educational content focused on a specific robotics topic
**Fields**:
- id: Unique identifier (e.g., "module-2")
- title: Display name (e.g., "The Digital Twin")
- description: Brief overview of module content
- prerequisites: List of required knowledge areas
- learning_objectives: Array of high-level learning goals
- chapters: Array of chapter references
- estimated_duration: Time to complete in hours

**Relationships**:
- Contains many Chapters
- May reference other Modules for cross-module learning

**Validation Rules**:
- Title must be 5-50 characters
- Learning objectives must be specific and measurable
- Prerequisites must reference existing content or external resources

## Entity: Chapter
**Description**: Educational unit with specific learning goals and hands-on exercises
**Fields**:
- id: Unique identifier within module (e.g., "digital-twin-fundamentals")
- title: Display name (e.g., "Digital Twin Fundamentals")
- module_id: Reference to parent module
- description: Brief overview of chapter content
- learning_objectives: Array of specific, measurable objectives (3-5 items)
- prerequisites: List of required knowledge (references to other chapters/sections)
- content_sections: Array of Section references
- exercises: Array of Exercise references
- estimated_duration: Time to complete in minutes
- difficulty_level: Enum (beginner, intermediate, advanced)

**Relationships**:
- Belongs to one Module
- Contains many Sections
- Contains many Exercises
- May reference other Chapters for cross-chapter learning

**Validation Rules**:
- Learning objectives must be specific, measurable, and achievable
- Difficulty level must align with prerequisites
- Estimated duration must be realistic based on content complexity

## Entity: Section
**Description**: Subdivision of chapters with focused content on a specific topic
**Fields**:
- id: Unique identifier within chapter
- title: Section heading
- chapter_id: Reference to parent chapter
- content_type: Enum (concept, example, tutorial, reference)
- content: Markdown text with educational material
- order: Integer for ordering within chapter
- learning_objectives: Array of specific objectives addressed
- code_examples: Array of code snippets (if applicable)

**Relationships**:
- Belongs to one Chapter
- May reference other Sections for cross-referencing

**Validation Rules**:
- Content must be educational and relevant to chapter objectives
- Code examples must be tested and functional
- Order must be unique within parent chapter

## Entity: Exercise
**Description**: Hands-on activity that allows students to practice concepts learned
**Fields**:
- id: Unique identifier within chapter
- title: Exercise name
- chapter_id: Reference to parent chapter
- description: Brief overview of what the exercise teaches
- objectives: Specific learning goals for the exercise
- prerequisites: Knowledge or setup required before starting
- steps: Array of ordered instructions
- expected_outcome: Description of what student should achieve
- difficulty_level: Enum (beginner, intermediate, advanced)
- estimated_duration: Time to complete in minutes

**Relationships**:
- Belongs to one Chapter
- May reference Sections for background information

**Validation Rules**:
- Steps must be clear and achievable
- Expected outcome must be measurable
- Difficulty must align with chapter difficulty

## Entity: Example
**Description**: Practical demonstration of concepts with code, commands, or procedures
**Fields**:
- id: Unique identifier
- title: Example name
- related_section_id: Reference to related Section (optional)
- example_type: Enum (code, command, procedure, configuration)
- content: The actual example (code, command text, or procedure)
- explanation: Text explaining what the example demonstrates
- expected_output: Description of what should happen when executed
- troubleshooting_notes: Common issues and solutions

**Relationships**:
- May belong to a Section
- May reference other Examples for progressive learning

**Validation Rules**:
- Content must be tested and functional
- Expected output must match actual behavior
- Troubleshooting notes should address common student errors

## Entity: LearningObjective
**Description**: Specific, measurable goal that defines what students should learn
**Fields**:
- id: Unique identifier
- text: The objective statement (follows SMART criteria)
- entity_type: Enum (module, chapter, section, exercise)
- entity_id: Reference to the entity it belongs to
- priority: Enum (must_have, should_have, could_have)
- assessment_method: How the objective will be evaluated

**Relationships**:
- Belongs to one entity (Module, Chapter, Section, or Exercise)

**Validation Rules**:
- Text must be specific, measurable, achievable, relevant, and time-bound
- Priority must align with entity importance
- Assessment method must be practical and achievable

## Entity: Resource
**Description**: External or internal reference material supporting educational content
**Fields**:
- id: Unique identifier
- title: Resource name
- url: Link to the resource
- resource_type: Enum (documentation, tutorial, video, paper, tool)
- relevance: Description of how it relates to content
- difficulty_level: Enum (beginner, intermediate, advanced)
- estimated_reading_time: For text-based resources
- verification_status: Enum (verified, unverified, outdated)

**Relationships**:
- May be referenced by many Sections, Chapters, or Exercises

**Validation Rules**:
- URL must be valid and accessible
- Verification status must be current
- Relevance must be clearly explained

## State Transitions

### Chapter States
- **Draft**: Initial state, content being developed
- **Review**: Content ready for review, awaiting approval
- **Published**: Content approved and available to students
- **Archived**: Content no longer current, but maintained for reference

### Exercise States
- **Design**: Exercise concept being developed
- **Development**: Exercise being built and tested
- **Testing**: Exercise being validated for accuracy
- **Ready**: Exercise complete and ready for use
- **Deprecated**: Exercise no longer relevant or functional

## Relationships Summary

```
Module (1) -----> (*) Chapter
Chapter (1) ----> (*) Section
Chapter (1) ----> (*) Exercise
Section (1) ----> (*) Example
Chapter (*) ----> (*) Resource
Exercise (*) ---> (*) Resource
```

## Validation Rules Summary

1. **Content Hierarchy**: All content must fit within the Module -> Chapter -> Section hierarchy
2. **Learning Alignment**: All content must support specific learning objectives
3. **Prerequisite Validation**: Content must not assume knowledge not covered in prerequisites
4. **Difficulty Consistency**: Content difficulty must be consistent within each level
5. **Resource Verification**: All external resources must be verified and current