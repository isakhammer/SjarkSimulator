OUTDIR := build
MAIN   := overview
FIGURE_SCRIPT := scripts/generate_figures.py

.PHONY: pdf clean figures

pdf: figures
	mkdir -p "$(OUTDIR)"
	latexmk -pdf -interaction=nonstopmode -halt-on-error -output-directory="$(OUTDIR)" "$(MAIN).tex"

figures:
	python3 "$(FIGURE_SCRIPT)"

clean:
	latexmk -C -output-directory="$(OUTDIR)" "$(MAIN).tex" || true
	rm -rf "$(OUTDIR)"
