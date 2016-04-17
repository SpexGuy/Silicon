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

bool applyNetDecomp;
bool useNetOrdering;

int main(int argc, char **argv)
{

    if(argc!=5){
        printf("Usage : %s -d=[0-1] -n=[0-1] <input_benchmark_name> <output_file_name> \n", argv[0]);
        return 1;
    }

    clock_t time, dt;
    int status;
    char *dFlag = argv[1];
    char *nFlag = argv[2];
    char *inputFileName = argv[3];
    char *outputFileName = argv[4];

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

    /// create a new routing instance
    RoutingInst rst;

	/// Read in the benchmark
    // resource cage for in:
    {   ifstream in(inputFileName);
        if (!in) {
            cerr << "Couldn't read from file " << inputFileName << endl;
            exit(1);
        }
		time = clock();
        status = readBenchmark(in, rst);
        if (status == 0) {
            printf("ERROR: reading input file \n");
            exit(1);
        }
		dt = clock() - time;
		cout << "Read " << rst.numNets << " nets into " << rst.numCells << " cells in " << (dt * 1000)/CLOCKS_PER_SEC << " ms." << endl;
    }

    /// Run actual routing
	time = clock();
    status = solveRouting(rst);
    if(status==0){
        printf("ERROR: running routing \n");
        exit(1);
    }
	dt = clock() - time;
	cout << "Routed in " << (dt * 1000)/CLOCKS_PER_SEC << " ms." << endl;

    int of = writeCongestionSvg(rst, "congestion.html");
    cout << "Total overflow: " << of << endl;

    /// Write the result
    // resource cage for out
    {	ofstream out(outputFileName);
		if (!out) {
			cerr << "Couldn't write to " << outputFileName << endl;
			exit(1);
		}
		time = clock();
		status = writeOutput(out, rst);
		if(status==0){
			printf("ERROR: writing the result \n");
			exit(1);
		}
		dt = clock() - time;
		cout << "Wrote output in " << (dt * 1000)/CLOCKS_PER_SEC << " ms." << endl;
    }

    printf("\nDONE!\n");
    exit(0);
}
