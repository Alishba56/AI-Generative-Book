<!--
Version change: old (N/A) → new (0.1.0)
List of modified principles:
  - Added Principle I: Accuracy
  - Added Principle II: Clarity
  - Added Principle III: Reproducibility
  - Added Principle IV: Simplicity
  - Added Principle V: Standards
Added sections:
  - Constraints
  - Success Criteria
Removed sections: N/A
Templates requiring updates:
  - .specify/templates/plan-template.md: ✅ updated (implicit, no direct change needed as it refers to constitution)
  - .specify/templates/spec-template.md: ✅ updated (implicit, no direct change needed as it refers to constitution)
  - .specify/templates/tasks-template.md: ✅ updated (implicit, no direct change needed as it refers to constitution)
  - .gemini/commands/sp.adr.toml: ✅ updated (reviewed, no changes needed)
  - .gemini/commands/sp.analyze.toml: ✅ updated (reviewed, no changes needed)
  - .gemini/commands/sp.checklist.toml: ✅ updated (reviewed, no changes needed)
  - .gemini/commands/sp.clarify.toml: ✅ updated (reviewed, no changes needed)
  - .gemini/commands/sp.constitution.toml: ✅ updated (reviewed, no changes needed)
  - .gemini/commands/sp.git.commit_pr.toml: ✅ updated (reviewed, no changes needed)
  - .gemini/commands/sp.implement.toml: ✅ updated (reviewed, no changes needed)
  - .gemini/commands/sp.phr.toml: ✅ updated (reviewed, no changes needed)
  - .gemini/commands/sp.plan.toml: ✅ updated (reviewed, no changes needed)
  - .gemini/commands/sp.specify.toml: ✅ updated (reviewed, no changes needed)
  - .gemini/commands/sp.tasks.toml: ✅ updated (reviewed, no changes needed)
Follow-up TODOs:
  - TODO(RATIFICATION_DATE): Original adoption date unknown.
Runtime guidance docs:
  - README.md: ⚠ pending (file not found in current directory structure)
  - docs/quickstart.md: ⚠ pending (file not found in current directory structure)
-->
# AI-Native Textbook on Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. Accuracy
All technical content MUST be verified against ROS 2, Gazebo, and NVIDIA Isaac documentation to ensure factual correctness and up-to-dateness.

### II. Clarity
Content MUST be written for computer science and robotics students, ensuring accessibility and ease of understanding for the target audience.

### III. Reproducibility
All instructions, particularly for simulations and code examples, MUST be clear and comprehensive enough for readers to replicate them successfully.

### IV. Simplicity
The textbook MUST consist of six chapters, maintaining conciseness and focus on core learning objectives.

### V. Standards
Content MUST adhere to academic and technical writing standards:
- Citation style: APA (where references needed).
- No plagiarism: All content must be original or properly attributed.
- Clear headings and code snippets: Structure and formatting must enhance readability.
- Markdown format: The book MUST be suitable for deployment on Docusaurus.

## Constraints

The project MUST adhere to the following constraints:
- Chapter Count: Exactly 6 chapters.
- Chapter Length: Each chapter MUST be between 1500–2500 words.
- Illustrations/Diagrams: Placeholder Markdown links MUST be included for all planned illustrations and diagrams.
- Implementation Scope: No front-end or backend implementation is required for this project.

## Success Criteria

The project will be considered successful if the following criteria are met:
- Student Understanding: Students MUST be able to understand the fundamental principles of Physical AI and the basics of humanoid robotics after reading the textbook.
- Setup and Simulation: Students MUST be able to successfully set up ROS 2 and simulate a simple humanoid robot by following the provided instructions.
- Deployment Readiness: The completed book MUST be ready for deployment on Docusaurus GitHub Pages.

## Governance
This Constitution outlines the foundational principles and guidelines for the "AI-Native Textbook on Physical AI & Humanoid Robotics" project.
- Amendments: Any amendments to this Constitution require a formal proposal, discussion, and approval by the project leads. All changes MUST be documented with a clear rationale and an incremented version.
- Compliance: All project contributions and artifacts MUST comply with the principles and guidelines set forth in this document. Regular reviews will assess adherence.
- Conflict Resolution: In cases of conflict between project practices and this Constitution, the Constitution SHALL take precedence.

**Version**: 0.2.0 | **Ratified**: TODO(RATIFICATION_DATE): Original adoption date unknown. | **Last Amended**: 2025-12-07