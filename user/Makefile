.PHONY: all clean

all: b3dfgutil b3dfgtest b3dfgstress capture_frames

b3dfgutil: b3dfgutil.c
	gcc $^ -lreadline -o $@

b3dfgtest: b3dfgtest.c
	gcc $^ -o $@

b3dfgstress: b3dfgstress.c
	gcc $^ -o $@

capture_frames: capture_frames.c
	gcc $^ -lb3dfg -o $@

clean:
	@rm -f b3dfgtest b3dfgutil *.o

