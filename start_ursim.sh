#!/bin/bash
docker rm -f ursim_quest >/dev/null 2>&1

docker run --rm -d --net host \
  --security-opt seccomp=unconfined \
  -e ROBOT_MODEL=UR5 \
  --name ursim_quest \
  universalrobots/ursim_cb3:latest

echo "--- waiting for URControl RUNNING ---"
for i in $(seq 1 20); do
  sleep 4
  m=$(docker exec ursim_quest bash -c "grep -oE 'robot_mode: [A-Z ]+-> [A-Z ]+' /ursim/URControl.log 2>/dev/null | tail -1")
  echo "  [$((i*4))s] $m"
  echo "$m" | grep -q "RUNNING" && break
done
echo "URSim is ready!"