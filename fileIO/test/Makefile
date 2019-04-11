#
# Compile and run thruster allocation examples
#

CC = gcc
CXX = g++
RM = rm -f

# -I"include file"
INCLUDEPATH=-I../../third_party/eigen \
			-I../include \

CPPFLAGS = -g -Wall -std=c++14 $(INCLUDEPATH)
#LDLIBS = $(shell root-config --libs) -lblas -llapack -larmadillo -lfftw3

# library file: -Lpath/to/library -l"name of library"


TARGETS = test

all: $(TARGETS)

clean:
	$(RM) ${TARGETS}.exe 

# %: %.cpp
# 	$(CXX) $(CPPFLAGS) -o $@.exe -march=native -O3 -mavx $< $(CSOURCES) $(LDLIBS)

%: %.cpp
	$(CXX) $(CPPFLAGS) -o $@.exe -march=native -O3 -mavx  $< 
%: %.c
	$(CC) $(CPPFLAGS) -o $@.exe -O3 $< $(LDLIBS)
