#!/bin/bash

spinner() {
    local pid=$!
    local delay=0.1
    local spinstr='|/-\'
    while [ "$(ps a | awk '{print $1}' | grep $pid)" ]; do
        local temp=${spinstr#?}
        printf " [%c]  " "$spinstr"
        local spinstr=$temp${spinstr%"$temp"}
        # sleep $delay
        printf "\r"
    done
    printf "    \b\b\b\b"
}

# Example of using the spinner with a sleep command
(sleep 5) &
spinner
