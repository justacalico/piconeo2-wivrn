#!/usr/bin/env bash
# Per-file line coverage of the tested library sources, via gcov.
# Usage: coverage.sh <build_dir> <cpp_dir>
set -u
BUILD="$1"
CPP="$2"

total_lines=0
total_hit=0

for gcda in $(find "$BUILD" -name '*.gcda' -path '*p2wivrn_testable*'); do
    src=$(gcov -n -o "$(dirname "$gcda")" "$gcda" 2>/dev/null | \
          awk '/^File/{f=$2} /^Lines executed/{print f, $0}' | head -1)
    [ -z "$src" ] && continue
    file=$(echo "$src" | awk '{print $1}' | sed 's|^File:||')
    pct=$(echo "$src" | grep -oP '\d+\.\d+(?=% of)')
    lines=$(echo "$src" | grep -oP '(?<=of )\d+')
    [ -z "$lines" ] && continue
    hit=$(echo "$pct $lines" | awk '{printf "%d", $1*$2/100}')
    printf "%6.2f%%  %5d  %s\n" "$pct" "$lines" "${file#$CPP/}"
    total_lines=$((total_lines + lines))
    total_hit=$((total_hit + hit))
done

if [ "$total_lines" -gt 0 ]; then
    echo "------------------------------------------------------------"
    awk -v h="$total_hit" -v t="$total_lines" \
        'BEGIN {printf "TOTAL %6.2f%%  %5d lines\n", h*100/t, t}'
fi
