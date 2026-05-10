CC = g++
SRCS = src/tracking_class.cpp

UNAME_S := $(shell uname -s)
PYTHONPATH := $(shell which python)

ifeq ($(UNAME_S),Linux)
    CFLAGS = -g -Wall -std=c++11 -O3 -Wall -fPIC -shared -std=c++11 `$(PYTHONPATH) -m pybind11 --includes` -I ./
    PROG = src/lib/find_marker.so
endif

ifeq ($(UNAME_S),Darwin)
    CFLAGS = -g -Wall -std=c++11 -O3 -Wall -shared -std=c++11 -undefined dynamic_lookup `$(PYTHONPATH) -m pybind11 --includes` -I ./
    PROG = src/lib/find_marker`$(PYTHONPATH)-config --extension-suffix`
endif

$(PROG):$(SRCS)
	mkdir -p src/lib
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS)

clean:
	rm -rf src/lib
