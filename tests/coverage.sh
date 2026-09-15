#!/usr/bin/env bash
# Per-file line coverage of the tested library sources, via gcov.
# Usage: coverage.sh <build_dir> <cpp_dir>
set -u
BUILD="$1"
CPP="$(realpath "$2")"

# Collect "File + Lines executed" pairs from every gcda, keep only files under
# the source tree, and merge headers shared across translation units.
report=$(mktemp)
for gcda in $(find "$BUILD" -name '*.gcda'); do
    gcov -n -o "$(dirname "$gcda")" "$gcda" 2>/dev/null
done | awk '
    /^File / { file = substr($0, 6); gsub(/'"'"'/, "", file); next }
    /^Lines executed:/ {
        match($0, /[0-9]+\.[0-9]+/); pct = substr($0, RSTART, RLENGTH) + 0;
        match($0, /of [0-9]+/);      lines = substr($0, RSTART + 3, RLENGTH - 3) + 0;
        if (lines > 0)
            print file "\t" pct "\t" lines;
    }
' > "$report"

# Per unique file keep the entry with the most counted lines (a header's best
# coverage across TUs), then print sorted by coverage ascending.
awk -F '\t' -v cpp="$CPP" '
    index($1, cpp) == 1 {
        f = substr($1, length(cpp) + 2);
        if ($2 > best_pct[f]) { best_lines[f] = $3; best_pct[f] = $2 }
    }
    END {
        for (f in best_lines) printf "%s\t%.2f\t%d\n", f, best_pct[f], best_lines[f];
    }
' "$report" | sort -t$'\t' -k2,2n > "$report.sorted"

total_lines=0
total_hit=0
while IFS=$'\t' read -r f pct lines; do
    hit=$(awk -v p="$pct" -v l="$lines" 'BEGIN {printf "%d", p * l / 100}')
    printf "%6.2f%%  %5d  %s\n" "$pct" "$lines" "$f"
    total_lines=$((total_lines + lines))
    total_hit=$((total_hit + hit))
done < "$report.sorted"

if [ "$total_lines" -gt 0 ]; then
    echo "------------------------------------------------------------"
    awk -v h="$total_hit" -v t="$total_lines" \
        'BEGIN {printf "TOTAL %6.2f%%  %5d lines\n", h * 100 / t, t}'
fi
rm -f "$report" "$report.sorted"
