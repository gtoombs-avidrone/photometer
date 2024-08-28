export

# Assumes the presence of MinGW
cxx=${mingw_home}/g++
cxxflags=-O2 -std=c++20 -Wall -march=native

all: reference.exe

%.exe: %.o
	$$cxx $$cxxflags -o $@ $<

%.o: %.cpp makefile
	$$cxx $$cxxflags -o $@ $< -c
