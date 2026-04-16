# Dev Container Profiles

This repository uses multiple profiles to avoid conflicts across different machines.

Available profiles:

- Generic AMD64: `.devcontainer/generic-amd64/devcontainer.json`
- NVIDIA AMD64: `.devcontainer/nvidia-amd64/devcontainer.json`
- Generic ARM64: `.devcontainer/generic-arm64/devcontainer.json`
- NVIDIA ARM64: `.devcontainer/nvidia-arm64/devcontainer.json`

## How to open the right profile

1. Open the project root in VS Code.
2. Run `Dev Containers: Reopen in Container`.
3. When the list appears, select the profile that matches your machine.

If the list does not appear, use `Dev Containers: Open Folder in Container...` from the project root.