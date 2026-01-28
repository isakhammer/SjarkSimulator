#!/usr/bin/env bash
set -euo pipefail

# Builds the latex document. The dependencies can be found in Dockerfile

LATEX_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${LATEX_DIR}/.." && pwd)"

MAIN="${MAIN:-main}"
GENERATE_FIGURES="${GENERATE_FIGURES:-1}"

if [[ "${GENERATE_FIGURES}" != "0" ]]; then
  python3 "${LATEX_DIR}/scripts/generate_figures.py"
fi

if command -v latexmk >/dev/null 2>&1; then
  (cd "${LATEX_DIR}" && latexmk -pdf -interaction=nonstopmode -halt-on-error "${MAIN}.tex")
  (cd "${LATEX_DIR}" && latexmk -c "${MAIN}.tex")
else
  (cd "${LATEX_DIR}" && pdflatex -interaction=nonstopmode -halt-on-error "${MAIN}.tex")
  (cd "${LATEX_DIR}" && pdflatex -interaction=nonstopmode -halt-on-error "${MAIN}.tex")
  rm -f "${LATEX_DIR}/${MAIN}.aux" "${LATEX_DIR}/${MAIN}.log" "${LATEX_DIR}/${MAIN}.out" \
    "${LATEX_DIR}/${MAIN}.toc" "${LATEX_DIR}/${MAIN}.lof" "${LATEX_DIR}/${MAIN}.fls" \
    "${LATEX_DIR}/${MAIN}.fdb_latexmk" "${LATEX_DIR}/${MAIN}.synctex.gz"
fi
