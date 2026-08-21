#!/usr/bin/env bash
set -euo pipefail

REMOTE_HOST="10.63.28.15"
REMOTE_USER="hub"
REMOTE_DIR="/home/hub/Projects"
BACKUP_DIR="${REMOTE_DIR}/backups"
LOCAL_PROJECT="hubcounter"
REMOTE_PROJECT="hub_counter"
DATE=$(date +%Y%m%d)
TARBALL="${REMOTE_PROJECT}_${DATE}.tar.gz"
LOCAL_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== Hub Counter Shop Deploy ==="

# 1. Create tarball of deployment files
echo "Packaging ${TARBALL}..."
PARENT_DIR="$(dirname "$LOCAL_DIR")"
TEMP_LINK="${PARENT_DIR}/${REMOTE_PROJECT}"

# Create temporary symlink if local and remote names differ
if [ "${LOCAL_PROJECT}" != "${REMOTE_PROJECT}" ]; then
    ln -s "${LOCAL_PROJECT}" "${TEMP_LINK}"
    trap "rm -f '${TEMP_LINK}'" EXIT
fi

tar -czf "/tmp/${TARBALL}" \
    -C "${PARENT_DIR}" \
    -h \
    --exclude='__pycache__' \
    --exclude='.pytest_cache' \
    --exclude='*.pyc' \
    --exclude='venv' \
    --exclude='.claude' \
    --exclude='hub_counter.egg-info' \
    --exclude='tests' \
    --exclude='docs' \
    --exclude='config.yaml' \
    "${REMOTE_PROJECT}"

# Clean up symlink
if [ "${LOCAL_PROJECT}" != "${REMOTE_PROJECT}" ]; then
    rm -f "${TEMP_LINK}"
fi

# 2. Copy tarball to remote
echo "Uploading to ${REMOTE_USER}@${REMOTE_HOST}..."
scp "/tmp/${TARBALL}" "${REMOTE_USER}@${REMOTE_HOST}:/tmp/${TARBALL}"

# 3. Remote: backup old code, deploy new code, restart service
echo "Deploying on remote..."
ssh "${REMOTE_USER}@${REMOTE_HOST}" bash -s <<EOF
set -euo pipefail

echo "  Backing up current code to ${BACKUP_DIR}/${TARBALL}..."
tar -czf "${BACKUP_DIR}/${TARBALL}" -C "${REMOTE_DIR}" "${REMOTE_PROJECT}"

echo "  Removing old code (preserving venv and config.yaml)..."
cd "${REMOTE_DIR}/${REMOTE_PROJECT}"
find . -mindepth 1 \
    ! -name 'venv' ! -path './venv/*' \
    ! -name 'config.yaml' \
    -delete 2>/dev/null || true

echo "  Extracting new code..."
tar -xzf "/tmp/${TARBALL}" -C "${REMOTE_DIR}" --strip-components=0

echo "  Restarting hub.service..."
sudo systemctl restart hub.service

echo "  Cleaning up remote tarball..."
rm -f "/tmp/${TARBALL}"

echo "  Service status:"
sudo systemctl status hub.service --no-pager -l || true
EOF

# 4. Clean up local tarball
rm -f "/tmp/${TARBALL}"

echo "=== Deploy complete ==="
