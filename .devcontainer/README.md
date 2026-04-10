# Dev Container Profiles

Questa repository usa profili multipli per evitare conflitti tra macchine diverse.

Profili disponibili:

- Intel AMD64: `.devcontainer/intel-amd64/devcontainer.json`
- NVIDIA AMD64: `.devcontainer/nvidia-amd64/devcontainer.json`
- Intel ARM64: `.devcontainer/intel-arm64/devcontainer.json`
- NVIDIA ARM64: `.devcontainer/nvidia-arm64/devcontainer.json`

## Come aprire il profilo giusto

1. Apri la root del progetto in VS Code.
2. Esegui `Dev Containers: Reopen in Container`.
3. Quando compare la lista, seleziona il profilo corretto per la macchina.

Se non compare la lista, usa `Dev Containers: Open Folder in Container...` dalla root del progetto.

In questo modo non devi piu modificare lo stesso file tra Intel e NVIDIA e `git status` non segnala cambiamenti inutili sul devcontainer.