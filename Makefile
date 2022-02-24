.PHONY: all clean

CC := g++-11
CFLAGS = `pkg-config --cflags --libs opencv4` -std=c++20 -Wno-deprecated-enum-enum-conversion

SRCS := $(wildcard *.cpp)
OBJS := $(SRCS:cpp=o)


all: main.out

# Link .o to main
main.out: $(OBJS) 
	$(CC) -o $@ $(OBJS) $(CFLAGS)
	./main.out

# Compile .cpp to .o
$(OBJS): %.o: %.cpp 
	$(CC) -c $< $(CFLAGS)

# Clean
clean:
	rm -f $(OBJS) main.out

