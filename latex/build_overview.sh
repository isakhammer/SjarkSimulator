#!/usr/bin/env bash
set -euo pipefail

# Builds the latex document. The dependencies can be found in Dockerfile

LATEX_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${LATEX_DIR}/.." && pwd)"

MAIN="${MAIN:-main}"
OUTDIR="${OUTDIR:-build}"
GENERATE_FIGURES="${GENERATE_FIGURES:-1}"

if [[ "${OUTDIR}" != /* ]]; then
  OUTDIR="${LATEX_DIR}/${OUTDIR}"
fi

if [[ "${GENERATE_FIGURES}" != "0" ]]; then
  python3 "${LATEX_DIR}/scripts/generate_figures.py"
fi

mkdir -p "${OUTDIR}"

if command -v latexmk >/dev/null 2>&1; then
  latexmk -pdf -interaction=nonstopmode -halt-on-error -output-directory="${OUTDIR}" "${LATEX_DIR}/${MAIN}.tex"
else
  pdflatex -interaction=nonstopmode -halt-on-error -output-directory="${OUTDIR}" "${LATEX_DIR}/${MAIN}.tex"
  pdflatex -interaction=nonstopmode -halt-on-error -output-directory="${OUTDIR}" "${LATEX_DIR}/${MAIN}.tex"
fi
