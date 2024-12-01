#!/bin/bash

progress_bar() {
    local progress=0
    local total=50  # total length of the bar
    while [ $progress -le $total ]; do
        bar=$(printf "%-${total}s" "#")
        printf "[%-${total}s] %d%%\r" "${bar:0:$progress}" $((progress * 2))
        sleep 0.1
        ((progress++))
    done
    echo ""
}

progress_bar
