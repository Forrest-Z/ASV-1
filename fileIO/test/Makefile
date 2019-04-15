#
# Compile and run thruster allocation examples
#

CC = gcc
CXX = g++
RM = rm -f

# library file: -Lpath/to/library -l"name of library"
HEADERDIR=../include \
          /home/scar1et/Coding/ASV/third_party/eigen  \
          /opt/mosek/8/tools/platform/linux64x86/h
MOSEKLIBDIR=/opt/mosek/8/tools/platform/linux64x86/bin

IPATHS=-I$(HEADERDIR)
LPATH=-L$(MOSEKLIBDIR) -Wl,-rpath-link,$(MOSEKLIBDIR) -Wl,-rpath=$(MOSEKLIBDIR)
LIBS=-lmosek64

# -I"include file"
INCLUDEPATH=-I../../third_party/eigen \
			-I../include \

CPPFLAGS = -g -Wall -std=c++14 $(INCLUDEPATH)
#LDLIBS = $(shell root-config --libs) -lblas -llapack -larmadillo -lfftw3

%.o: %.cpp
	$(CXX) -c -g $(IPATHS) -o $@ $<
testutility: testutility.o
	$(CXX) -g $(LPATH) -o $@ $< $(LIBS)
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
