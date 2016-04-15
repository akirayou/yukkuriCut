CXXFLAGS=-O0 -ggdb

all:test test_deadVol

test:test.cc acl_cut2.ino config.h deadvolume.ino
	$(CXX) $(CXXFLAGS) -o test test.cc

test_deadVol:test_deadVol.cc acl_cut2.ino config.h
	$(CXX) $(CXXFLAGS) -o test_deadVol test_deadVol.cc
