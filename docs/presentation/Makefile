TEXFILES := $(shell ls *.tex)
PDFFILES := $(TEXFILES:%.tex=%.pdf)
PDFFLAGS := -interaction=batchmode -halt-on-error
PDFFLAGS := -halt-on-error

all: $(PDFFILES)

presentation.pdf:presentation.tex
	pdflatex $(PDFFLAGS) $<
	pdflatex $(PDFFLAGS) $<

%.pdf:%.dot
	dot -Tpdf $< -o $@

