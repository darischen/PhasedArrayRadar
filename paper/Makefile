.PHONY: pdf clean

pdf:
	latexmk -pdf -interaction=nonstopmode paper.tex

clean:
	latexmk -c
	rm -f paper.pdf *.figlist *.makefile

distclean: clean
	rm -f *.pdf
