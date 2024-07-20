for i in {1..50}; do
    printf "\033[42m\033[30m Progress: [%-${i}s] %d%%\r" "#" "$i"
    sleep 0.1
done
echo -e "\033[0m"  # Reset formatting
