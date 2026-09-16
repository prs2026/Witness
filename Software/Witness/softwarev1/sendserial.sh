while true; do
    read -e -p "> " line
    history -s "$line"
    printf '%s\r\n' "$line" > /dev/ttyACM0
done
