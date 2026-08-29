
#!/bin/bash
set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}Building Kamikaze Drone with Smart Caching${NC}"

# Check if base image exists
if docker image inspect kamikaze-drone-base:latest >/dev/null 2>&1; then
    echo -e "${GREEN}✓ Base image found - using cache${NC}"
else
    echo -e "${YELLOW}⚠ Base image not found - building from scratch (10-15 min)${NC}"
    echo -e "${YELLOW}This only happens once or when dependencies change${NC}"
    
    docker build --platform linux/amd64 \
        -f Dockerfile.base \
        -t kamikaze-drone-base:latest \
        --cache-from kamikaze-drone-base:latest \
        .
fi

# Build main image (fast - only code changes)
echo -e "${GREEN}Building main image (usually 1-2 min)...${NC}"
docker build --platform linux/amd64 \
    -t kamikaze-drone:latest \
    --cache-from kamikaze-drone:latest \
    --cache-from kamikaze-drone-base:latest \
    .

echo -e "${GREEN}✓ Build complete!${NC}"
echo ""
echo "⏱ Build times:"
echo "  - First build: 10-15 minutes"
echo "  - Code changes: 1-2 minutes"
echo "  - Dependency changes: 10-15 minutes"