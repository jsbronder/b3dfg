KERNELDIR := /lib/modules/`uname -r`/build
module := b3dfg
PACKAGE=b3dfg-0.6_rc1

.PHONY: module clean

module:
	B3DFG_DEBUG=y make -C $(KERNELDIR) M=`pwd`

nodebug:
	make -C $(KERNELDIR) M=`pwd`

clean:
	make -C $(KERNELDIR) M=`pwd` $@

dist:
	mkdir $(PACKAGE)
	cp Makefile *.c Kbuild $(PACKAGE)
	sed -i -e 's/B3DFG_DEBUG=y//g' $(PACKAGE)/Makefile
	tar -cjf $(PACKAGE).tar.bz2 $(PACKAGE)
	rm -rf $(PACKAGE)

