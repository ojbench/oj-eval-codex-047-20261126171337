#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
assets=/workspace/data/047/第三次机考B卷下发文件
scratch=$(mktemp -d)
trap 'rm -rf "$scratch"' EXIT
cp "$assets"/src/*.h "$assets"/src/main.cpp "$scratch/"
cp "$repo/src.hpp" "$scratch/"
g++ -std=c++17 -O2 -Wall -Wextra "$scratch/main.cpp" -o "$scratch/run"
for input in "$assets"/testcases/*.in; do
    (cd "$scratch" && ./run < "$input" 2> result)
    echo "$(basename "$input")"
    cat "$scratch/result"
    echo
    if rg -q 'Collision|Speeding|Didn.t end' "$scratch/result"; then
        exit 1
    fi
done
