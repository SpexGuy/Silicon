SYSTEM     = x86-64_sles10_4.1
LIBFORMAT  = static_pic

# ---------------------------------------------------------------------         
# Compiler selection                                                            
# ---------------------------------------------------------------------         

CCC = g++

# ---------------------------------------------------------------------         
# Compiler options                                                              
# ---------------------------------------------------------------------         

CCOPT = -I ./include -m64 -O -fPIC -fexceptions -DNDEBUG -DIL_STD -g -Wall -std=c++11

# ---------------------------------------------------------------------         
# Link options and libraries                                                    
# ---------------------------------------------------------------------         

CCFLAGS = $(CCOPT)
CCLNFLAGS = -lm -pthread

#------------------------------------------------------------                   
#  make all      : to compile.                                     
#  make execute  : to compile and execute.                         
#------------------------------------------------------------    

all: ROUTE.exe

ROUTE.exe: main.o ece556.o svg.o obj
	rm -f ROUTE.exe
	$(CCC) $(LINKFLAGS) $(CCFLAGS) main.o ece556.o svg.o $(shell find obj -type f) $(CCLNFLAGS) -o ROUTE.exe

main.o: main.cpp ece556.h svg.h
	rm -f main.o
	$(CCC) $(CCFLAGS) main.cpp -c

ece556.o: ece556.cpp ece556.h include/flute
	rm -f ece556.o
	$(CCC) $(CCFLAGS) ece556.cpp -c

svg.o: svg.cpp svg.h ece556.h
	rm -f svg.o
	$(CCC) $(CCFLAGS) svg.cpp -c

obj: include/flute
	rm -rf obj/
	cd libraries/flute-3.1 \
	 && make
	mkdir obj/
	cp libraries/flute-3.1/*.o obj/

include/flute:
	rm -rf include/flute/
	mkdir -p include/flute/
	cp libraries/flute-3.1/*.h include/flute/
	cp libraries/flute-3.1/*.dat .


clean:
	rm -f *~ *.o ROUTE.exe

cleanall: clean
	rm -rf include/
	rm -rf obj/
