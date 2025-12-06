# Quick Build Guide

## Prerequisites

**You need Node.js installed:**
- Download: https://nodejs.org/
- Install the LTS version (v20.x or newer)
- Verify: Open PowerShell and run `node --version`

## Build Steps

### Option 1: Use Build Script (Easiest)

```cmd
cd vscode-extension
build.bat
```

This will:
1. Install dependencies
2. Compile TypeScript
3. Create `fiaphy-language-support-1.0.0.vsix`

### Option 2: Manual Commands

```bash
cd vscode-extension
npm install
npm run compile
npm run package
```

## Install the Extension

After building, you'll have a `.vsix` file. Install it:

```bash
code --install-extension fiaphy-language-support-1.0.0.vsix
```

Or in VS Code:
1. Press `Ctrl+Shift+P`
2. Type "Install from VSIX"
3. Select the `.vsix` file

## Troubleshooting

**Error: npm not found**
- Install Node.js from https://nodejs.org/
- Restart your terminal after installation

**Error: vsce not found**
- The build script will install it automatically
- Or manually: `npm install -g @vscode/vsce`

**Error: TypeScript compilation failed**
- Delete `node_modules` folder
- Run `npm install` again

## What Gets Created

- `fiaphy-language-support-1.0.0.vsix` - The extension package
- `out/` folder - Compiled JavaScript
- `node_modules/` folder - Dependencies

## Distribution

Share the `.vsix` file with users, or publish to marketplace.
