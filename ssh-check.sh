#!/bin/bash
echo "=== SSH Setup Status ==="

if [ -f "/home/ros/.ssh/id_rsa" ] || [ -f "/home/ros/.ssh/id_ed25519" ]; then
    echo "✓ SSH keys detected and mounted"
    echo ""
    echo "Testing GitLab connection..."
    ssh -T git@gitlab.com 2>&1 | head -1
    echo ""
    echo "Testing GitHub connection..."
    ssh -T git@github.com 2>&1 | head -1
else
    echo "⚠ No SSH keys mounted"
    echo ""
    echo "To enable SSH git operations, run with:"
    echo "  docker run -v ~/.ssh:/home/ros/.ssh your-image-name"
    echo ""
    echo "Current SSH directory contents:"
    ls -la /home/ros/.ssh/ 2>/dev/null || echo "SSH directory is empty"
fi

echo "======================="
