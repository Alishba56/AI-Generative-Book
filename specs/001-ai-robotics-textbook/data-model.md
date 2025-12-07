# Conceptual Data Model: AI-Native Robotics Textbook Content

## Entities

### Chapter
- **ID**: Unique identifier (e.g., `chapter-1`, `chapter-2`)
- **Title**: Chapter title (e.g., "The Robotic Nervous System")
- **Word Count**: Integer, actual word count (expected: 1500-2500)
- **Sections**: List of Section IDs
- **Summary**: Brief overview of chapter content
- **Exercises**: List of Exercise IDs

### Section
- **ID**: Unique identifier (e.g., `chapter-1-1`, `chapter-2-3`)
- **Title**: Section title (e.g., "ROS 2 Core Concepts")
- **Content**: Markdown text, including explanations, code snippets, illustrations
- **Illustrations**: List of Illustration IDs (e.g., Markdown image links)
- **Code Snippets**: List of Code Snippet IDs (referenced directly in Content)
- **References**: List of Reference IDs (APA style)

### Illustration
- **ID**: Unique identifier (e.g., `robot-node-architecture`)
- **Alt Text**: Descriptive text for accessibility and context
- **Placeholder URL**: URL to a placeholder image (e.g., `https://via.placeholder.com/600x400?text=Diagram+Placeholder`)
- **Final Asset Path**: Path to the actual image asset (to be determined during content creation)

### Code Snippet
- **ID**: Unique identifier (e.g., `python-rclpy-agent`)
- **Language**: Programming language (e.g., `python`, `cpp`)
- **Content**: Code block
- **Verification Status**: Boolean, indicating if snippet has been tested/verified

### Reference
- **ID**: Unique identifier (e.g., `ros2-docs-rclpy`)
- **APA Entry**: Full APA-formatted citation
- **URL**: Link to the source

### Exercise
- **ID**: Unique identifier
- **Question**: Text of the exercise question
- **Type**: (e.g., conceptual, practical, coding)
- **ChapterID**: Parent Chapter ID

## Relationships

- **Chapter has many Sections** (one-to-many)
- **Section has many Illustrations** (one-to-many)
- **Section has many Code Snippets** (one-to-many)
- **Section has many References** (one-to-many)
- **Chapter has many Exercises** (one-to-many)
