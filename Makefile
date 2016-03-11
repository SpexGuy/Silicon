SYSTEM     = x86-64_sles10_4.1
LIBFORMAT  = static_pic

# ---------------------------------------------------------------------         
# Compiler selection                                                            
# ---------------------------------------------------------------------         

CCC = g++

# ---------------------------------------------------------------------         
# Compiler options                                                              
# ---------------------------------------------------------------------         

CCOPT = -I ./include -L ./lib -m64 -O -fPIC -fexceptions -DNDEBUG -DIL_STD -g -Wall -std=c++11

# ---------------------------------------------------------------------         
# Link options and libraries                                                    
# ---------------------------------------------------------------------         

CCFLAGS = $(CCOPT) 
CCLNFLAGS = -lm -pthread -lboost_system -lboost_filesystem

#------------------------------------------------------------                   
#  make all      : to compile.                                     
#  make execute  : to compile and execute.                         
#------------------------------------------------------------    

all: ROUTE.exe

ROUTE.exe: main.o ece556.o lib obj
	rm -f ROUTE.exe
	$(CCC) $(LINKFLAGS) $(CCFLAGS) main.o ece556.o $(shell find obj -type f) $(CCLNFLAGS) -o ROUTE.exe

main.o: main.cpp ece556.h include/boost
	rm -f main.o
	$(CCC) $(CCFLAGS) main.cpp -c

ece556.o: ece556.cpp ece556.h include/boost include/flute
	rm -f ece556.o
	$(CCC) $(CCFLAGS) ece556.cpp -c

lib: include/boost

include/boost:
	rm -rf include/boost/
	rm -rf lib/
	cd libraries/boost_1_60_0 \
	 && ./bootstrap.sh --prefix=../.. --with-libraries=filesystem \
	 && ./b2 link=static install

obj: include/flute
	rm -rf obj/
	cd libraries/flute-3.1 \
	 && make
	mkdir obj/
	cp libraries/flute-3.1/*.o obj/

include/flute:
	rm -rf include/flute/
	mkdir include/flute/
	cp libraries/flute-3.1/*.h include/flute/
	cp libraries/flute-3.1/*.dat .


clean:
	rm -f *~ *.o ROUTE.exe

cleanall: clean
	rm -rf include/
	rm -rf lib/
	rm -rf obj/
