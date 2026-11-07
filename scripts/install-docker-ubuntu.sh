#!/usr/bin/env bash
# Reviewed Ubuntu 22.04 amd64-only installer. Invoked via pkexec after GUI confirmation.
# No downloaded shell code; packages come from Docker's signed official repository.
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo 'Administrator authorization required'; exit 1; }
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] || { echo 'Only Ubuntu 22.04 supported'; exit 1; }
[[ "$(dpkg --print-architecture)" == amd64 ]] || { echo 'Only amd64 supported'; exit 1; }
for package in docker.io docker-compose docker-compose-v2 docker-doc podman-docker containerd runc; do
  if dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -q 'install ok installed'; then
    echo "Conflicting package $package detected. Resolve manually; nothing removed."; exit 1
  fi
done
apt-get update
apt-get install -y ca-certificates curl
install -m 0755 -d /etc/apt/keyrings
# Do not replace existing sources or keys from another setup.
[[ ! -e /etc/apt/keyrings/docker.asc && ! -e /etc/apt/sources.list.d/docker.sources && ! -e /etc/apt/sources.list.d/docker.list ]] || { echo 'Docker source/key already exists; review existing setup manually.'; exit 1; }
curl --fail --show-error --silent --location --proto '=https' https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
chmod a+r /etc/apt/keyrings/docker.asc
cat > /etc/apt/sources.list.d/docker.sources <<'SOURCES'
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: jammy
Components: stable
Architectures: amd64
Signed-By: /etc/apt/keyrings/docker.asc
SOURCES
apt-get update
apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
systemctl start docker
printf '%s\n' 'Docker installed. No user group memberships were changed.'
