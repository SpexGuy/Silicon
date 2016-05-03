// ECE556 - Copyright 2014 University of Wisconsin-Madison.  All Rights Reserved.

#include <iostream>
#include <fstream>

#include "ece556.h"
#include "svg.h"

using std::ifstream;
using std::ofstream;
using std::cout;
using std::cerr;
using std::endl;

int main(int argc, char **argv)
{
    time_t start_time = time(nullptr);


    /// Validate arguments
    if(argc!=5){
        printf("Usage : %s -d=[0-1] -n=[0-1] <input_benchmark_name> <output_file_name> \n", argv[0]);
        return 1;
    }

    char *dFlag = argv[1];
    char *nFlag = argv[2];
    char *inputFileName = argv[3];
    char *outputFileName = argv[4];
    bool applyNetDecomp;
    bool useNetOrdering;

    if(dFlag[1] != 'd' || nFlag[1] != 'n' || dFlag[0] != '-' || nFlag[0] != '-'
       || dFlag[2] != '=' || nFlag[2] != '='){
        printf("Usage : %s -d=[0-1] -n=[0-1] <input_benchmark_name> <output_file_name> \n", argv[0]);
        return 1;
    }

    if(!(dFlag[3] == '1' || dFlag[3] == '0') || !(nFlag[3] == '1' || nFlag[3] == '0')){
        printf("Usage : %s -d=[0-1] -n=[0-1] <input_benchmark_name> <output_file_name> \n", argv[0]);
        return 1;
    }

    if(dFlag[3] == '1'){
      printf("Using net decomposition\n");
      applyNetDecomp = true;
    }else{
      printf("Not using net decomposition\n");
      applyNetDecomp = false;
    }

    if(nFlag[3] == '1'){
      printf("Using net ordering\n");
      useNetOrdering = true;
    }else{
      printf("Not using net ordering\n");
      useNetOrdering = false;
    }


    // Here's a time macro for timing things!
    clock_t dt;
    #define TIME(cmd) do {clock_t time = clock(); cmd; dt = clock() - time;} while(0)


    /// create a new routing instance
    RoutingInst rst;


	/// Read in the benchmark
    ifstream in(inputFileName);

    TIME(readBenchmark(in, rst));
    cout << "Read " << rst.numNets << " nets into " << rst.numCells << " cells in " << (dt * 1000)/CLOCKS_PER_SEC << " ms." << endl;

    in.close();


    /// Run actual routing
    time_t end_time = start_time;
    if (useNetOrdering)
        end_time += 30*60; // add 30 minutes max for RUARR (it will stop 1 minutes early if possible)

    TIME(solveRouting(rst, end_time, !applyNetDecomp));
    cout << "Routed in " << (dt * 1000)/CLOCKS_PER_SEC << " ms." << endl;


#ifndef NDEBUG
    /// Output a debug svg of the final solution
    int of = writeCongestionSvg(rst, "congestion.html");
    cout << "Total overflow: " << of << endl;
#endif


    /// Write the result
    ofstream out(outputFileName);

    TIME(writeOutput(out, rst));
    cout << "Wrote output in " << (dt * 1000)/CLOCKS_PER_SEC << " ms." << endl;

    printf("\nDONE!\n");
    exit(0); // Fuck RAII

    #undef TIME // scoped macros FTW
}
