design.pdf: design.dvi
	dvipdf $^

design.dvi: design.tex
	texi2dvi $^

clean:
	rm -f design.dvi design.pdf design.log design.aux
