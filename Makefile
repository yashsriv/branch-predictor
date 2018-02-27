# Author: Jared Stark;   Created: Mon Aug 16 11:28:20 PDT 2004
# Description: Makefile for building a cbp submission.

CXX = clang++

CFLAGS = -g -Wall
CXXFLAGS = -g -Wall -std=c++11

objects = cbp_inst.o main.o op_state.o predictor.o tread.o

predictor : $(objects)
	$(CXX) -o $@ $(objects)

cbp_inst.o : cbp_inst.h cbp_assert.h cbp_fatal.h cond_pred.h finite_stack.h indirect_pred.h stride_pred.h value_cache.h
main.o : tread.h cbp_inst.h predictor.h op_state.h
op_state.o : op_state.h
predictor.o : predictor.h op_state.h tread.h cbp_inst.h
tread.o : tread.h cbp_inst.h op_state.h

run: predictor
	./predictor traces/without-values/DIST-INT-1
	./predictor traces/without-values/DIST-INT-2
	./predictor traces/without-values/DIST-INT-3
	./predictor traces/without-values/DIST-MM-1
	./predictor traces/without-values/DIST-MM-2
	./predictor traces/without-values/DIST-SERV-1
	./predictor traces/without-values/DIST-SERV-2
	./predictor traces/without-values/DIST-SERV-3
	./predictor traces/without-values/DIST-SERV-4
	./predictor traces/without-values/DIST-SERV-5

.PHONY : clean
clean :
	rm -f predictor $(objects)

