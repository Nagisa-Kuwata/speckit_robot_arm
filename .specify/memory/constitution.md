# SpecKit Robot Arm Constitution

## Core Principles

### I. Rust First
All development must be done using the Rust programming language. This ensures memory safety, concurrency, and performance, which are critical for the project's goals.

### II. Embedded & Real-Time Performance
The project is designed for embedded systems development.
- **Real-Time Execution**: Ensure deterministic behavior and minimal latency.
- **Performance**: Optimize for high execution speed and efficient resource usage.
- **Accuracy**: Prioritize calculation precision and correctness, especially for robot arm kinematics and control logic.

### III. Iterative Git Workflow
Maintain a clean and traceable history.
- **Commit & Push**: Perform a git commit and push at the end of every development phase or logical unit of work.
- **Commit Messages**: Write clear and descriptive commit messages explaining the "what" and "why" of changes.

## Development Standards

### Code Quality
- Follow Rust idioms and best practices (clippy is your friend).
- Ensure all code is documented clearly.

### Testing
- Unit tests are required for all core logic.
- Integration tests should verify the interaction between components.

## Governance

This constitution serves as the primary guideline for all development within the SpecKit Robot Arm project.
All contributions must adhere to these principles.

**Version**: 1.0.0 | **Ratified**: 2026-02-27 | **Last Amended**: 2026-02-27
