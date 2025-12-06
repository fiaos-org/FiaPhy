# FiaPhy VS Code Extension - Build Instructions

## Quick Start

```bash
cd vscode-extension
npm install
npm run compile
npm run package
code --install-extension fiaphy-language-support-1.0.0.vsix
```

## What This Extension Does

1. **Syntax Highlighting**
   - FiaPhy namespace and classes highlighted
   - Deprecated functions shown in red
   - Core functions get special colors

2. **IntelliSense**
   - Type `FiaPhy::` and get auto-complete
   - Shows function signatures
   - Links to documentation

3. **Error Detection**
   - Warns about deprecated functions from v1.0.0
   - Detects missing `configure()` calls
   - Warns if `isFrameReady()` not checked

4. **Code Snippets**
   - Type `fiaphy-basic` for full template
   - Type `fiaphy-compute` for radiation computation pattern
   - All snippets have tab stops for easy editing

5. **Hover Documentation**
   - Hover over any FiaPhy function
   - See signature, description, example
   - Click link to open GitHub docs

## Files Created

```
vscode-extension/
├── package.json           - Extension manifest
├── src/
│   └── extension.ts      - Main extension logic
├── syntaxes/
│   └── fiaphy.tmLanguage.json - Syntax rules
├── snippets/
│   └── fiaphy.json       - Code snippets
├── tsconfig.json         - TypeScript config
├── README.md             - User documentation
└── .gitignore           - Git ignore rules
```

## Testing

1. Open `vscode-extension` folder in VS Code
2. Press F5 to launch Extension Development Host
3. Create new `.cpp` or `.ino` file
4. Type `#include <FiaPhy.h>`
5. Try snippets: `fiaphy-basic` + Tab

## Publishing to VS Code Marketplace

### Prerequisites

1. Create account at https://marketplace.visualstudio.com/
2. Create Azure DevOps organization
3. Get Personal Access Token (PAT)

### Steps

```bash
# Install vsce
npm install -g @vscode/vsce

# Login (one-time)
vsce login fiaos-org

# Package
vsce package

# Publish
vsce publish
```

## Customization

### Add More Snippets

Edit `snippets/fiaphy.json`:
```json
"Your Snippet Name": {
  "prefix": "your-prefix",
  "body": ["line 1", "line 2"],
  "description": "What it does"
}
```

### Add More Hover Docs

Edit `src/extension.ts`, function `getFiaPhyDocumentation()`:
```typescript
'yourFunction': {
    signature: 'void yourFunction()',
    description: 'What it does',
    url: 'https://...',
    example: 'yourFunction();'
}
```

### Add More Error Checks

Edit `src/extension.ts`, function `validateFiaPhyCode()`.

## Troubleshooting

**Extension not activating?**
- Check you have a `.cpp`, `.c`, or `.ino` file open
- Check VS Code version >= 1.80.0

**Snippets not working?**
- Make sure language mode is "C++" (bottom right corner)
- Restart VS Code after installing

**No syntax highlighting?**
- Check theme supports custom scopes
- Try Dark+ or Light+ theme

## Distribution Options

1. **VSIX File** - Share .vsix file, users install manually
2. **GitHub Releases** - Attach .vsix to release
3. **VS Code Marketplace** - Official publishing (requires Azure account)
4. **Open VSX** - Alternative marketplace (free)

---

Created for FiaPhy v1.0.1
Copyright (c) 2025 FiaOS.org
