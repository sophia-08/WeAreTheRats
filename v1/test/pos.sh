#!/bin/bash

for i in {1..40}; do
    printf "\033[2J"  # Clear screen
    printf "\033[%d;%dH" $((i % 10 + 1)) $((i * 2))  # Move cursor
    echo "Moving Text"
    sleep 0.1
done
