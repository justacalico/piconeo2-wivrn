#!/bin/bash
# Trigger the GitHub build workflow and block while streaming its output, so
# the GitLab job duration tracks the GitHub run and the build log shows up in
# the job trace as if it were a native runner.
set -euo pipefail

REPO="justacalico/piconeo2-wivrn"
WORKFLOW="build.yml"

REF="${1:-main}"
PUSH_REF="${2:-}"
CHANNEL="${CHANNEL:-release}"

if [ -n "$PUSH_REF" ]; then
  echo "Pushing $PUSH_REF to GitHub branch $REF..."
  git remote add github "git@github.com:$REPO.git" 2>/dev/null || true
  git remote update github
  git push -f github "$PUSH_REF:refs/heads/$REF"
fi

TS=$(( $(date +%s) - 60 ))
echo "Triggering $WORKFLOW @ $REF (channel: $CHANNEL)"
gh workflow run "$WORKFLOW" -R "$REPO" --ref "$REF" -f channel="$CHANNEL"

echo "Looking for run ID..."
RUN_ID=""
for i in $(seq 30); do
  sleep 5
  RUN_ID=$(gh api "repos/$REPO/actions/runs?branch=$REF&event=workflow_dispatch&per_page=5" \
    --jq ".workflow_runs | map(select((.created_at|fromdateiso8601) > $TS)) | .[0].id" \
    2>/dev/null || true)
  [ -n "$RUN_ID" ] && [ "$RUN_ID" != "null" ] && break
done

if [ -z "$RUN_ID" ] || [ "$RUN_ID" = "null" ]; then
  echo "Could not find GitHub run for $REF" >&2
  exit 1
fi

echo "Watching GitHub run $RUN_ID..."
CONCLUSION="failure"
for attempt in 1 2 3; do
  if gh run watch "$RUN_ID" -R "$REPO" --exit-status 2>&1; then
    CONCLUSION="success"
    break
  fi

  # watch can drop on transient API errors; check the real conclusion before
  # giving up
  C=$(gh api "repos/$REPO/actions/runs/$RUN_ID" -q '.conclusion' 2>/dev/null || true)
  case "$C" in
    success)
      CONCLUSION="success"; break ;;
    failure|cancelled|timed_out|startup_failure|action_required|stale)
      CONCLUSION="$C"; break ;;
    *)
      echo "gh run watch lost connection (attempt $attempt), retrying..."
      sleep 10 ;;
  esac
done

echo "GitHub run finished: $CONCLUSION"
[ "$CONCLUSION" = "success" ]
