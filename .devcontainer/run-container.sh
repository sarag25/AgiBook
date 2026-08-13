#!/bin/bash
# Avvia un container Docker normale (non devcontainer) sul workspace SmartRobotics_C-O.
#
# Da eseguire DA WSL Ubuntu, non da PowerShell: il socket X11 di WSLg
# (/tmp/.X11-unix) esiste solo nel contesto Linux, e lanciando da Windows
# Gazebo/RViz non troverebbero il display.
#
#   bash "/mnt/c/Users/<utente>/.../SmartRobotics_C-O/.devcontainer/run-container.sh"
#
# Nome container e immagine sono sovrascrivibili:
#   CONTAINER_NAME=ros2_test IMAGE=ros2_gui:v0.1 bash run-container.sh
set -e

CONTAINER_NAME="${CONTAINER_NAME:-ros2_smart}"
# v0.2 = snapshot del layer scrivibile di ros2_smart (docker commit), preso
# quando il container e' stato ricreato per togliere il mount di /etc/localtime.
IMAGE="${IMAGE:-smartrobotics:v0.2}"
# Volume per la config di Claude Code (login + server MCP). Senza questo la
# config vive nel layer scrivibile del container e sparisce a ogni ricreazione.
CLAUDE_VOLUME="${CLAUDE_VOLUME:-smartrobotics_claude}"

# Path derivati dalla posizione dello script: niente path assolute hardcoded.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BRAIN_DIR="$(dirname "$PROJECT_DIR")/RobotBrain"

# Container già esistente: lo riavvio invece di ricrearlo, così non perdo
# nulla di quello che c'e' nel suo layer scrivibile.
if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    echo "Container '$CONTAINER_NAME' esiste gia': lo avvio."
    docker start "$CONTAINER_NAME" >/dev/null
    echo "Pronto -> docker exec -it $CONTAINER_NAME bash"
    exit 0
fi

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "Immagine '$IMAGE' non trovata. Creala con:"
    echo "  docker build -t $IMAGE - < \"$SCRIPT_DIR/DockerFile\""
    exit 1
fi

# Il fuso si passa come variabile, NON montando /etc/localtime: Docker Desktop
# non fa bind diretto dei file da WSL, li copia in una cache interna
# (/run/desktop/mnt/host/wsl/docker-desktop-bind-mounts/) che si svuota al
# riavvio di Docker/WSL. Al docker start successivo il file non c'e' piu' e il
# container muore con "error mounting ... /etc/localtime: no such file or
# directory". Le cartelle vengono ricreate al volo, i file no.
HOST_TZ="$(cat /etc/timezone 2>/dev/null || echo Europe/Rome)"

EXTRA_ARGS=()
[ -d /dev/dri ] && EXTRA_ARGS+=(--device=/dev/dri)
if [ -d "$BRAIN_DIR" ]; then
    EXTRA_ARGS+=(-v "$BRAIN_DIR:/home/robot/RobotBrain")
else
    echo "Nota: '$BRAIN_DIR' non trovata, monto solo il progetto."
fi

docker run -dit --name "$CONTAINER_NAME" \
    --privileged \
    --network=host \
    --shm-size=4g \
    --group-add video \
    -w /home/robot/SmartRobotics \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e HOME=/home/robot \
    -e ROS_DISTRO=jazzy \
    -e LANG=C.UTF-8 \
    -e LC_ALL=C.UTF-8 \
    -e TZ="$HOST_TZ" \
    -e CLAUDE_CONFIG_DIR=/home/robot/.claude-config \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$CLAUDE_VOLUME:/home/robot/.claude-config" \
    -v "$PROJECT_DIR:/home/robot/SmartRobotics" \
    "${EXTRA_ARGS[@]}" \
    "$IMAGE" bash >/dev/null

# Un volume Docker appena creato appartiene a root, ma il container gira come
# robot (uid 1000): senza questo chown Claude Code non puo' scrivere la config.
docker exec -u root "$CONTAINER_NAME" chown -R robot:robot /home/robot/.claude-config

# Stesso setup della postCreateCommand del devcontainer, cosi' i due ambienti
# si comportano allo stesso modo.
docker exec "$CONTAINER_NAME" bash -c '
    grep -qxF "source /opt/ros/jazzy/setup.bash" ~/.bashrc \
        || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    grep -qxF "alias smartbuild=\"source ~/SmartRobotics/build.sh\"" ~/.bashrc \
        || echo "alias smartbuild=\"source ~/SmartRobotics/build.sh\"" >> ~/.bashrc
'

echo "Container '$CONTAINER_NAME' creato."
echo "  progetto : $PROJECT_DIR -> /home/robot/SmartRobotics"
echo "  entra    : docker exec -it $CONTAINER_NAME bash"
