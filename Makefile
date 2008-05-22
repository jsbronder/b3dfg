KERNELDIR := /lib/modules/`uname -r`/build
module := b3dfg

.PHONY: module clean

module:
	B3DFG_DEBUG=y make -C $(KERNELDIR) M=`pwd`

nodebug:
	make -C $(KERNELDIR) M=`pwd`

clean:
	make -C $(KERNELDIR) M=`pwd` $@

