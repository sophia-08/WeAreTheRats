#!/bin/bash

frames=(
  "     O     "
  "    O      "
  "   O       "
  "  O        "
  " O         "
  "O          "
  " O         "
  "  O        "
  "   O       "
  "    O      "
)

while true; do
  for frame in "${frames[@]}"; do
    printf "\r%s" "$frame"
    sleep 0.1
  done
done
