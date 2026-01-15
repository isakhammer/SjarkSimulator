OUTDIR := build
MAIN   := overview
FIGURE_SCRIPT := scripts/generate_figures.py

.PHONY: pdf clean figures

pdf: figures
	mkdir -p "$(OUTDIR)"
	@if command -v latexmk >/dev/null 2>&1; then \
		latexmk -pdf -interaction=nonstopmode -halt-on-error -output-directory="$(OUTDIR)" "$(MAIN).tex"; \
	else \
		pdflatex -interaction=nonstopmode -halt-on-error -output-directory="$(OUTDIR)" "$(MAIN).tex"; \
		pdflatex -interaction=nonstopmode -halt-on-error -output-directory="$(OUTDIR)" "$(MAIN).tex"; \
	fi

figures:
	python3 "$(FIGURE_SCRIPT)"

clean:
	latexmk -C -output-directory="$(OUTDIR)" "$(MAIN).tex" || true
	rm -rf "$(OUTDIR)"
