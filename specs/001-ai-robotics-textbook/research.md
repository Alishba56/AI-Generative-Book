# Research for AI-Native Robotics Textbook

## Decision: ROS 2 Version Recommendation
**Rationale**: Selecting a stable, widely-adopted, and long-term supported ROS 2 distribution is crucial for students to have a consistent and future-proof learning experience. Given the textbook's publication date, a recent LTS (Long Term Support) release is preferred.
**Alternatives Considered**:
- ROS 2 Foxy Fitzroy (EOL in May 2023): Too old, not recommended for new projects.
- ROS 2 Galactic Geochelone (EOL in Nov 2022): Too old.
- ROS 2 Humble Hawksbill (LTS, EOL in May 2027): Stable, excellent documentation, widely used.
- ROS 2 Iron Irwini (Non-LTS, EOL in Nov 2024): Newer features, but shorter support window.
- ROS 2 Jazzy Jalisco (LTS, EOL in May 2029 - expected): Potentially too new at the time of writing, might lack mature tooling/documentation.
**Decision**: Recommend **ROS 2 Humble Hawksbill** as the primary distribution due to its LTS status, maturity, and broad community support, ensuring a stable environment for the target audience. Mention that newer LTS versions like Jazzy Jalisco might be available and can be considered with minor adaptations.

## Decision: Gazebo Version Recommendation
**Rationale**: Gazebo simulation version should align well with the chosen ROS 2 distribution for seamless integration.
**Alternatives Considered**:
- Gazebo Classic: Older versions, less integrated with ROS 2 ecosystem.
- Gazebo Fortress (used with ROS 2 Humble): Direct compatibility.
- Gazebo Garden (used with ROS 2 Iron): Compatible with newer ROS 2.
**Decision**: Recommend **Gazebo Fortress** for direct compatibility with ROS 2 Humble Hawksbill.

## Decision: NVIDIA Isaac Sim Version Recommendation
**Rationale**: Isaac Sim offers advanced physics and photorealistic rendering beneficial for humanoid robotics. Compatibility with ROS 2 and Python versions is key.
**Alternatives Considered**: Specific older versions or non-Isaac simulators (e.g., PyBullet, MuJoCo).
**Decision**: Recommend the latest stable version of **NVIDIA Isaac Sim** that officially supports ROS 2 Humble Hawksbill and Python 3.x, to be identified at the time of content creation. Explicitly note system requirements.

## Decision: Python Version for ROS 2 rclpy and OpenAI Whisper
**Rationale**: ROS 2 distributions typically target specific Python versions. OpenAI libraries also support modern Python versions.
**Alternatives Considered**: Sticking to older Python versions (not recommended).
**Decision**: Use the **Python 3.x version officially supported by ROS 2 Humble Hawksbill** (e.g., Python 3.8/3.10), ensuring compatibility for `rclpy` and the OpenAI Whisper library.

## Decision: Diagram Placeholder Strategy
**Rationale**: Illustrations are crucial for clarity, but creating them during the planning phase is premature. A clear placeholder strategy ensures they are integrated into the Markdown structure and tracked.
**Alternatives Considered**: Embed dummy images directly (adds unnecessary size), leave comments only (less structured).
**Decision**: Use **Markdown image links with descriptive alt text and a clear placeholder image URL** (e.g., `![Alt Text for Diagram](https://via.placeholder.com/600x400?text=Diagram+Placeholder)`). This visually indicates the missing diagram and provides context.

## Decision: APA Citation in Markdown
**Rationale**: Adhering to APA style is a constitution standard. Integrating it into Markdown effectively is key for Docusaurus deployment.
**Alternatives Considered**: Custom Markdown extensions (adds complexity), simple inline text (lacks formal structure).
**Decision**: Employ **standard Markdown links for URLs to sources** and provide **manual APA-style inline citations** (e.g., `(Author, Year)`) where direct references are needed within the text. A dedicated "References" section at the end of each chapter will list full APA-formatted entries.
