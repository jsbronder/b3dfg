KERNELDIR := /lib/modules/`uname -r`/build
module := b3dfg

.PHONY: module clean

module:
	make -C $(KERNELDIR) M=`pwd`

clean:
	make -C $(KERNELDIR) M=`pwd` $@

