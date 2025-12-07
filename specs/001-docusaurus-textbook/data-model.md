# Data Model: Physical AI & Humanoid Robotics Textbook

## Entities

### Textbook Chapter
- **Fields**:
  - id: string (unique identifier for the chapter)
  - title: string (chapter title)
  - week: number (which week this chapter belongs to, 1-13)
  - position: number (position within the week, 1-2 for most weeks)
  - learningObjectives: string[] (list of learning objectives)
  - prerequisites: string[] (prerequisites for this chapter)
  - theoryContent: string (500-1000 words of theoretical content)
  - codeExamples: CodeExample[] (2-3 code examples)
  - exercises: string[] (hands-on exercises)
  - summary: string (chapter summary)
  - hardwareRequirements: string[] (if applicable to the chapter)
  - relatedChapters: string[] (IDs of related chapters)

### CodeExample
- **Fields**:
  - id: string (unique identifier)
  - language: string (python, yaml, xml, etc.)
  - purpose: string (what this code example demonstrates)
  - setupInstructions: string (how to set up to run this code)
  - code: string (the actual code content)
  - expectedOutput: string (what output to expect)
  - comments: string[] (inline comments explaining key parts)

### NavigationItem
- **Fields**:
  - id: string (unique identifier)
  - title: string (display title)
  - path: string (URL path to the content)
  - parent: string (parent navigation item, if any)
  - children: NavigationItem[] (child navigation items)
  - order: number (order in navigation hierarchy)

### ImageAsset
- **Fields**:
  - id: string (unique identifier)
  - filename: string (original filename)
  - optimizedFilename: string (filename after optimization)
  - size: number (file size in bytes)
  - altText: string (accessibility description)
  - category: string (week-01, week-02, etc.)
  - usageContext: string (which chapter(s) this image is used in)

## Relationships

- TextbookChapter contains multiple CodeExample instances
- TextbookChapter contains multiple ImageAsset instances
- NavigationItem forms a hierarchical tree structure
- TextbookChapter has one-to-many relationship with NavigationItem (through sidebar organization)

## Validation Rules

- TextbookChapter.title is required and must be 5-100 characters
- TextbookChapter.week must be between 1-13
- TextbookChapter.theoryContent must be between 500-1000 words
- TextbookChapter.codeExamples must have 2-3 items
- CodeExample.language must be one of: python, yaml, xml, urdf
- ImageAsset.size must be less than 500KB (512000 bytes)
- NavigationItem.order must be unique within its parent

## State Transitions

- TextbookChapter: draft → review → approved → published
- CodeExample: created → tested → validated → ready
- ImageAsset: uploaded → optimized → validated → ready