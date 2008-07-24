.PHONY: all clean

all: b3dfgutil b3dfgtest

b3dfgutil: b3dfgutil.c
	gcc $^ -lreadline -o $@

b3dfgtest: b3dfgtest.c
	gcc $^ -o $@

clean:
	@rm -f b3dfgtest b3dfgutil *.o

