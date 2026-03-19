#!/bin/bash
# Update script for Go2-Dynamic-Inspection repository
# This script pulls the latest changes and rebuilds all workspaces

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Go2 Planner Suite Update Script ===${NC}"
echo ""

# Navigate to repository root
cd "$ROOT_DIR"

# Check for uncommitted changes
if ! git diff-index --quiet HEAD -- 2>/dev/null; then
    echo -e "${YELLOW}Warning: You have uncommitted changes.${NC}"
    echo "Uncommitted files:"
    git status --short
    echo ""
    read -p "Do you want to stash these changes? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Stashing local changes..."
        git stash push -m "Auto-stash before update $(date +%Y-%m-%d_%H:%M:%S)"
        STASHED=true
    else
        echo "Continuing without stashing..."
        STASHED=false
    fi
else
    STASHED=false
fi

# Fetch remote changes
echo ""
echo -e "${GREEN}Fetching remote changes...${NC}"
git fetch origin

# Check if local branch is behind remote
LOCAL=$(git rev-parse @)
REMOTE=$(git rev-parse @{u} 2>/dev/null || echo "")
BASE=$(git merge-base @ @{u} 2>/dev/null || echo "")

if [ -z "$REMOTE" ]; then
    echo -e "${YELLOW}No upstream branch set. Using origin/main...${NC}"
    git pull origin main
elif [ "$LOCAL" = "$REMOTE" ]; then
    echo -e "${GREEN}Already up to date!${NC}"
    if [ "$STASHED" = true ]; then
        echo "Restoring stashed changes..."
        git stash pop
    fi
    exit 0
elif [ "$LOCAL" = "$BASE" ]; then
    echo -e "${GREEN}Pulling latest changes...${NC}"
    git pull
else
    echo -e "${RED}Diverged from remote. Please resolve manually.${NC}"
    exit 1
fi

# Restore stashed changes
if [ "$STASHED" = true ]; then
    echo ""
    echo "Restoring stashed changes..."
    if git stash pop; then
        echo -e "${GREEN}Successfully restored stashed changes${NC}"
    else
        echo -e "${RED}Conflict while restoring stashed changes. Please resolve manually.${NC}"
        echo "Your changes are saved in the stash. Run 'git stash list' to see them."
        exit 1
    fi
fi

# Ask user if they want to rebuild
echo ""
read -p "Do you want to rebuild all workspaces? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo -e "${GREEN}Rebuilding all workspaces...${NC}"
    
    # Ask for build type
    read -p "Use release build with optimizations? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        # Detect number of CPU cores
        NCORES=$(nproc)
        echo "Building in Release mode with $NCORES parallel workers..."
        "$SCRIPT_DIR/build_all.sh" --release --parallel-workers "$NCORES"
    else
        echo "Building in Debug mode..."
        "$SCRIPT_DIR/build_all.sh"
    fi
    
    echo ""
    echo -e "${GREEN}=== Update Complete! ===${NC}"
    echo "All workspaces have been rebuilt."
else
    echo ""
    echo -e "${YELLOW}Skipping rebuild.${NC}"
    echo "Note: You may need to rebuild manually if dependencies changed:"
    echo "  ./scripts/build_all.sh"
fi

echo ""
echo -e "${GREEN}Repository updated successfully!${NC}"
