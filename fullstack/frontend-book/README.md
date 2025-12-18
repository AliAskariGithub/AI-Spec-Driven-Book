# AI-Spec Driven Book - Frontend

This is the frontend for the AI-Spec Driven Book, a comprehensive educational resource for AI and Computer Science students learning about robotics, specifically focusing on ROS 2 (Robot Operating System 2) as middleware for humanoid robot control.

## Overview

The AI-Spec Driven Book provides educational content that explains the purpose and architecture of robotic middleware for humanoid robot control. It teaches core robotic communication concepts including nodes, topics, and services with practical examples, and helps users learn how to implement communication nodes using appropriate programming interfaces for AI-to-controller communication.

## Features

- **Module-based Learning**: Organized into progressive modules starting with fundamental concepts
- **Practical Examples**: Real-world examples demonstrating robotic middleware concepts
- **AI Integration**: Focus on connecting AI agents to robot systems
- **Humanoid Robotics**: Specialized content for humanoid robot models and control

## Getting Started

### Prerequisites

- Node.js (version 18 or higher)
- npm or yarn package manager
- Git for version control

### Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd frontend-book
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

### Development

To start the development server:
```bash
npm start
```

This will start a local development server at `http://localhost:3000` with hot reloading.

### Building for Production

To build the static site for production:
```bash
npm run build
```

The built site will be available in the `build/` directory.

### Serving the Built Site

To serve the built site locally for testing:
```bash
npm run serve
```

## Project Structure

- `docs/` - Contains all the educational content in Markdown format
- `src/` - Custom React components and CSS
- `static/` - Static assets like images
- `docusaurus.config.js` - Main Docusaurus configuration
- `sidebars.js` - Navigation sidebar configuration

## Modules

### Module 1: The Robotic Nervous System

This module covers:
1. Robotic Middleware Fundamentals
2. Core Communication Concepts
3. AI Agents and Robot Models

## Contributing

To contribute content or improvements:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## Deployment

This site is designed to be deployed to GitHub Pages. The configuration in `docusaurus.config.js` includes settings for GitHub Pages deployment.

## License

This project is licensed under the MIT License.
