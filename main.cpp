// ECE556 - Copyright 2014 University of Wisconsin-Madison.  All Rights Reserved.

#include <iostream>
#include <fstream>

#include "ece556.h"

using std::ifstream;
using std::ofstream;
using std::cout;
using std::cerr;
using std::endl;

int main(int argc, char **argv)
{

    if(argc!=3){
        printf("Usage : %s <input_benchmark_name> <output_file_name> \n", argv[0]);
        return 1;
    }

	clock_t time, dt;
    int status;
    char *inputFileName = argv[1];
    char *outputFileName = argv[2];

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
