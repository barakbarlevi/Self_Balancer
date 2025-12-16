#!/bin/bash

PI_USER="barak"
PI_HOST="10.100.102.53"
PI_PATH="~"

DEST="."

FILES=(
    "Offline_telemetry.csv"
)

echo "Fetching files from $PI_USER@$PI_HOST:$PI_PATH ..."
for FILE in "${FILES[@]}"; do
    echo " → $FILE"
    scp ${PI_USER}@${PI_HOST}:${PI_PATH}/${FILE} "${DEST}/"
done

echo "Done!"

