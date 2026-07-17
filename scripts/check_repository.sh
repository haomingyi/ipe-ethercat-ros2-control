#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${project_dir}"

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "Not inside a Git repository: ${project_dir}" >&2
  exit 1
fi

forbidden_pattern='(^|/)(build[^/]*|install[^/]*|log[^/]*|artifacts|archives|\.idea|\.vscode)/'
forbidden_files="$(git ls-files | grep -E "${forbidden_pattern}" || true)"
if [[ -n "${forbidden_files}" ]]; then
  echo "Generated, private, or workstation files are tracked:" >&2
  printf '%s\n' "${forbidden_files}" >&2
  exit 1
fi

secret_pattern='gho_[[:alnum:]_]+|github_pat_[[:alnum:]_]+|BEGIN (RSA|OPENSSH|EC) PRIVATE KEY'
secret_matches="$(git grep -n -E "${secret_pattern}" -- . ':(exclude)ipe/SOEM/**' || true)"
if [[ -n "${secret_matches}" ]]; then
  echo "Possible credential material found in tracked files:" >&2
  printf '%s\n' "${secret_matches}" >&2
  exit 1
fi

echo "Repository boundary check passed."
echo "Tracked files: $(git ls-files | wc -l)"
echo "Branch: $(git branch --show-current)"
git status -sb
