SYSTEM     = x86-64_sles10_4.1
LIBFORMAT  = static_pic

# ---------------------------------------------------------------------         
# Compiler selection                                                            
# ---------------------------------------------------------------------         

CCC = g++

# ---------------------------------------------------------------------         
# Compiler options                                                              
# ---------------------------------------------------------------------         

CCOPT = -I ./include -L ./lib -m64 -O -fPIC -fexceptions -DNDEBUG -DIL_STD -g -Wall

# ---------------------------------------------------------------------         
# Link options and libraries                                                    
# ---------------------------------------------------------------------         

CCFLAGS = $(CCOPT) 
CCLNFLAGS = -lm -pthread -lboost_filesystem

#------------------------------------------------------------                   
#  make all      : to compile.                                     
#  make execute  : to compile and execute.                         
#------------------------------------------------------------    

ROUTE.exe: main.o ece556.o 
	/bin/rm -f ROUTE.exe
	$(CCC) $(LINKFLAGS) $(CCFLAGS) main.o ece556.o $(CCLNFLAGS) -o ROUTE.exe

main.o: main.cpp ece556.h include/boost
	/bin/rm -f main.o
	$(CCC) $(CCFLAGS) main.cpp -c

ece556.o: ece556.cpp ece556.h include/boost
	/bin/rm -f ece556.o
	$(CCC) $(CCFLAGS) ece556.cpp -c

include/boost:
	cd libraries/boost_1_60_0 \
	 && ./bootstrap.sh --prefix=../.. --with-libraries=filesystem \
	 && ./b2 link=static install

clean:
	/bin/rm -f *~ *.o ROUTE.exe

cleanall: clean
	/bin/rm -rf include/
	/bin/rm -rf lib/
