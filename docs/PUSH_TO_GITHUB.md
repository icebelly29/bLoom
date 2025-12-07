# Pushing to GitHub - Quick Guide

## Step 1: Set Git User Info (if not already set)

```bash
git config user.name "Your Name"
git config user.email "your.email@example.com"
```

## Step 2: Commit All Changes

```bash
cd ~/Desktop/bloom
git add .
git commit -m "Initial commit: Hand holding detector with auto-start and improved detection"
```

## Step 3: Create GitHub Repository

1. Go to https://github.com/new
2. Create a new repository (e.g., `bloom` or `hand-holding-detector`)
3. **DO NOT** initialize with README, .gitignore, or license (we already have these)
4. Copy the repository URL (e.g., `https://github.com/yourusername/bloom.git`)

## Step 4: Add Remote and Push

```bash
# Add remote (replace with your actual repository URL)
git remote add origin https://github.com/yourusername/bloom.git

# Rename branch to main if needed
git branch -M main

# Push to GitHub
git push -u origin main
```

## Alternative: Using SSH

If you prefer SSH:

```bash
git remote add origin git@github.com:yourusername/bloom.git
git push -u origin main
```

## If You Get Authentication Errors

### Option 1: Personal Access Token
1. Go to GitHub Settings → Developer settings → Personal access tokens
2. Generate a new token with `repo` permissions
3. Use token as password when prompted

### Option 2: SSH Key
1. Generate SSH key: `ssh-keygen -t ed25519 -C "your.email@example.com"`
2. Add to GitHub: Settings → SSH and GPG keys → New SSH key
3. Use SSH URL for remote

## Verify Push

After pushing, check your GitHub repository to verify all files are there.

## Files Included

The following important files are included:
- ✅ `hand_holding_detector.py` - Main detection script
- ✅ `MCbloomMaster5.ino` - Arduino master sketch
- ✅ `MCbloomSlave5.ino` - Arduino slave sketch
- ✅ `README.md` - Project overview
- ✅ `DEVELOPMENT_GUIDE.md` - Comprehensive development guide
- ✅ `SETUP_AUTOSTART.md` - Auto-start setup instructions
- ✅ `HAND_DETECTION_COMPARISON.md` - MediaPipe comparison
- ✅ `ISSUES_FOUND.md` - Known issues and fixes
- ✅ Configuration files and scripts

## Files Excluded (via .gitignore)

- ❌ `node_modules/` - Node dependencies
- ❌ `__pycache__/` - Python cache
- ❌ `binaries/` - Compiled Arduino binaries
- ❌ `bin/` - Binary tools
- ❌ Log files and temporary files

