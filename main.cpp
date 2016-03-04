// ECE556 - Copyright 2014 University of Wisconsin-Madison.  All Rights Reserved.

#include <boost/filesystem.hpp>
#include <iostream>

#include "ece556.h"

using std::ifstream;
using std::ofstream;
using std::cout;
using std::cerr;
using std::endl;

int main(int argc, char **argv)
{
    boost::filesystem::path full_path( boost::filesystem::current_path() );
    std::cout << "Hangin' in " << full_path << std::endl;
    std::cout << "adaptec1.gr size: " << boost::filesystem::file_size("adaptec1.gr") << std::endl;

    if(argc!=3){
        printf("Usage : %s <input_benchmark_name> <output_file_name> \n", argv[0]);
        return 1;
    }

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
		clock_t time = clock();
        status = readBenchmark(in, rst);
        if (status == 0) {
            printf("ERROR: reading input file \n");
            exit(1);
        }
		clock_t dt = clock() - time;
		cout << "Read " << rst.numNets << " nets into " << rst.numCells << " cells in " << (dt * 1000)/CLOCKS_PER_SEC << " ms.";
    }

    /// Run actual routing
    status = solveRouting(rst);
    if(status==0){
        printf("ERROR: running routing \n");
        exit(1);
    }

    /// Write the result
    // resource cage for out
    {	ofstream out(outputFileName);
		if (!out) {
			cerr << "Couldn't write to " << outputFileName << endl;
			exit(1);
		}
		status = writeOutput(out, rst);
		if(status==0){
			printf("ERROR: writing the result \n");
			exit(1);
		}
    }

    printf("\nDONE!\n");
    exit(0);
}
