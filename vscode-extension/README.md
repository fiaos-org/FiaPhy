# FiaPhy VS Code Extension

Language support extension for FiaPhy DTDSS library.

## Features

- **Syntax Highlighting**: FiaPhy classes and functions highlighted
- **IntelliSense**: Auto-completion for FiaPhy API
- **Error Detection**: Warns about deprecated functions and common mistakes
- **Hover Documentation**: Hover over functions to see docs
- **Code Snippets**: Quick templates for common patterns
- **Direct Links**: Links to GitHub documentation

## Installation

### From VSIX File

```bash
cd vscode-extension
npm install
npm run compile
npm run package
```

Install the generated `.vsix` file:
```
code --install-extension fiaphy-language-support-1.0.0.vsix
```

### From Marketplace (After Publishing)

1. Open VS Code
2. Press `Ctrl+Shift+X` (Extensions)
3. Search "FiaPhy"
4. Click Install

## Usage

### Snippets

Type these prefixes and press Tab:

- `fiaphy-basic` - Complete setup with dual sensors
- `fiaphy-config` - Configure location
- `fiaphy-ref` - Feed reference data
- `fiaphy-flux` - Feed flux data
- `fiaphy-compute` - Compute with frame check
- `fiaphy-thermo` - Thermodynamics functions

### Commands

- `FiaPhy: Open Documentation` - Opens GitHub docs
- `FiaPhy: Insert Basic Example` - Inserts starter code

### Error Detection

Extension warns about:
- Deprecated function names (v1.0.0 → v1.0.1 migration)
- Missing `configure()` call
- Missing `isFrameReady()` check

## Publishing to Marketplace

1. Create Azure DevOps account
2. Get Personal Access Token
3. Login: `vsce login fiaos-org`
4. Publish: `vsce publish`

## Development

```bash
npm install
npm run watch
```

Press F5 to debug extension.

## License

Copyright (c) 2025 FiaOS.org
