# Physical AI & Humanoid Robotics Textbook

Welcome to the **Physical AI & Humanoid Robotics Textbook** project! This repository contains the source code for the interactive textbook built with [Docusaurus](https://docusaurus.io/).

## 🚀 Getting Started

Follow these instructions to set up the project locally.

### Prerequisites

- **Node.js** (version 18 or higher)
- **npm** (comes with Node.js)

### Installation

1.  Clone the repository (if you haven't already):
    ```bash
    git clone https://github.com/YOUR_USERNAME/Physical-AI-Humanoid-Robotics-Textbook.git
    cd Physical-AI-Humanoid-Robotics-Textbook
    ```

2.  Navigate to the `docusaurus` directory:
    ```bash
    cd docusaurus
    ```

3.  Install dependencies:
    ```bash
    npm install
    ```

## 📖 Running the Book Locally

To start the development server and view the book in your browser:

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

Local URL: `http://localhost:3000`

## ✍️ How to Add Content

1.  **Navigate to the docs folder**:
    All content files are located in `docusaurus/docs/`.

2.  **Create Markdown files**:
    Add your chapters or pages as `.md` files.

    Example structure:
    ```
    docs/
    ├── intro.md
    ├── module-1/
    │   ├── chapter-1.md
    │   └── chapter-2.md
    └── ...
    ```

3.  **Preview changes**:
    The development server (`npm start`) will automatically reload to show your new content.

## 🏗️ Building for Production

To generate static files for deployment:

```bash
npm run build
```

The build artifacts will be stored in the `build/` directory.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📝 License

This project is licensed under the MIT License.
