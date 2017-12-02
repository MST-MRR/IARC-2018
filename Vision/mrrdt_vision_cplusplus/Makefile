CXX = g++
CUDA_HOME=/usr/local/cuda-8.0
PYTHON_LIB_DIR=/usr/lib/python3.5/config-3.5m-x86_64-linux-gnu/

BOOST_LIB=/usr/lib/x86_64-linux-gnu
BOOST_PYTHON_FLAGS +=-L$(PYTHON_LIB_DIR) -L$(BOOST_LIB) -lpython3.5 -lboost_python-py35
CXXFLAGS +=-std=c++11 -c -Wall $(shell pkg-config --cflags opencv) -I. -I$(CUDA_HOME)/include -I/usr/local/include -I /usr/include/python3.5m
LDFLAGS += $(shell pkg-config --libs opencv) -lboost_system -lboost_filesystem $(BOOST_PYTHON_FLAGS) -L$(CUDA_HOME)/lib64 -lcuda -lcudart
CUDAFLAGS +=--ptxas-options=-v --compiler-options '-fPIC'

all:
	$(CXX) $(CXXFLAGS) -fPIC -c roomba.cpp -o roomba.o
	nvcc $(CUDAFLAGS) -c kernel.cu -o kernel.o
	$(CXX) -shared roomba.o kernel.o -o threshold_gpu.so $(LDFLAGS)

clean:
	rm -f *.o threshold_gpu