OUTDIR := build
MAIN   := main
FIGURE_SCRIPT := scripts/generate_figures.py

.PHONY: pdf clean figures pdf-with-figures

pdf:
	mkdir -p "$(OUTDIR)"
	@if command -v latexmk >/dev/null 2>&1; then \
		latexmk -pdf -interaction=nonstopmode -halt-on-error -output-directory="$(OUTDIR)" "$(MAIN).tex"; \
	else \
		pdflatex -interaction=nonstopmode -halt-on-error -output-directory="$(OUTDIR)" "$(MAIN).tex"; \
		pdflatex -interaction=nonstopmode -halt-on-error -output-directory="$(OUTDIR)" "$(MAIN).tex"; \
	fi

pdf-with-figures: figures pdf

figures:
	python3 "$(FIGURE_SCRIPT)"

clean:
	latexmk -C -output-directory="$(OUTDIR)" "$(MAIN).tex" || true
	rm -rf "$(OUTDIR)"
