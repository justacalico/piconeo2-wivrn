#!/usr/bin/env bash
# Per-file line coverage of the tested library sources, via gcov.
# A line counts as covered if it was hit in ANY translation unit (headers get
# instantiated per-TU, so taking a single TU's numbers undercounts).
# Usage: coverage.sh <build_dir> <cpp_dir>
set -u
BUILD="$(realpath "$1")"
CPP="$(realpath "$2")"

WORK=$(mktemp -d)
i=0
for gcda in $(find "$BUILD" -name '*.gcda'); do
    i=$((i + 1))
    mkdir -p "$WORK/$i"
    (cd "$WORK/$i" && gcov -o "$(dirname "$gcda")" "$gcda" >/dev/null 2>&1)
done

# Merge every generated .gcov: file -> line -> covered-in-any-TU.
find "$WORK" -name '*.gcov' -print0 | sort -z | xargs -0 cat | awk -v cpp="$CPP" '
    /^ *-: *0:Source:/ {
        file = substr($0, index($0, "Source:") + 7);
        cur = (index(file, cpp) == 1) ? substr(file, length(cpp) + 2) : "";
        next;
    }
    /^ *[0-9$*#=-]+:/ {
        # gcov format: count:lineno:source. - = dead line, ##### = not run.
        split($0, a, ":");
        count = a[1]; lineno = a[2] + 0; gsub(/^[ \t]+|[ \t]+$/, "", count);
        rest = substr($0, index($0, ":") + 1);
        src = substr(rest, index(rest, ":") + 1);
        if (cur != "" && lineno > 0 && count != "-") {
            total[cur SUBSEP lineno] = 1;
            if (count != "#####" && count != "=====")
                hit[cur SUBSEP lineno] = 1;
            srcline[cur SUBSEP lineno] = src;
        }
        next;
    }
    END {
        for (k in total) {
            split(k, b, SUBSEP); f = b[1]; ln = b[2] + 0;
            if (f ~ /^third_party\//) continue;
            if (srcline[k] ~ /__builtin_unreachable/) continue;
            # gcov attaches unreachable counters to lone closing/opening
            # tokens; a never-called function still shows up on real lines.
            if (srcline[k] ~ /^[ \t]*[}{);,]+[ \t]*$/) continue;
            t[f]++;
            # A ##### line that does not end a statement (no semicolon) is gcov
            # mis-attributing a multi-line call: count it covered when the next
            # counted line ran. Complete statements stay uncovered.
            covered = (k in hit);
            if (!covered && srcline[k] !~ /;/) {
                for (n = ln + 1; n <= ln + 15; ++n)
                    if ((f SUBSEP n) in total) {
                        covered = ((f SUBSEP n) in hit);
                        break;
                    }
            }
            if (covered)
                h[f]++;
            else
                uncov[f] = uncov[f] " " ln;
        }
        for (f in t)
            printf "%.2f\t%d\t%s\t%s\n", h[f] * 100 / t[f], t[f], f, uncov[f];
    }
' | sort -t$'\t' -k1,1n > "$WORK/report"

total_lines=0
total_hit=0
while IFS=$'\t' read -r pct lines f uncov; do
    hit=$(awk -v p="$pct" -v l="$lines" 'BEGIN {printf "%d", p * l / 100}')
    printf "%6.2f%%  %5d  %s  (uncovered:%s)\n" "$pct" "$lines" "$f" "$uncov"
    total_lines=$((total_lines + lines))
    total_hit=$((total_hit + hit))
done < "$WORK/report"

if [ "$total_lines" -gt 0 ]; then
    echo "------------------------------------------------------------"
    awk -v h="$total_hit" -v t="$total_lines" \
        'BEGIN {printf "TOTAL %6.2f%%  %5d lines\n", h * 100 / t, t}'
fi
rm -rf "$WORK"
