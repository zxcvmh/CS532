#!/bin/bash
# Script chạy Dashboard Backend trên Laptop (CS532)
# Tự động trỏ sang run_laptop.sh để đảm bảo đồng nhất

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
exec "$DIR/run_laptop.sh" "$@"
