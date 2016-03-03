// ECE556 - Copyright 2014 University of Wisconsin-Madison.  All Rights Reserved.

#include <boost/filesystem.hpp>
#include <iostream>

#include "ece556.h"

int main(int argc, char **argv)
{
	boost::filesystem::path full_path( boost::filesystem::current_path() );
	std::cout << "Hangin' in " << full_path << std::endl;
	std::cout << "main.cpp size: " << boost::filesystem::file_size("main.cpp") << std::endl;

	if(argc!=3){
 		printf("Usage : %s <input_benchmark_name> <output_file_name> \n", argv[0]);
 		return 1;
 	}

 	int status;
	char *inputFileName = argv[1];
 	char *outputFileName = argv[2];

 	/// create a new routing instance
 	routingInst rst;
	
 	/// read benchmark
 	status = readBenchmark(inputFileName, &rst);
 	if(status==0){
 		printf("ERROR: reading input file (%d)\n", status);
 		return 1;
 	}
	
 	/// run actual routing
 	status = solveRouting(&rst);
 	if(status==0){
 		printf("ERROR: running routing (%d)\n", status);
 		return 1;
 	}
	
 	/// write the result
 	status = writeOutput(outputFileName, &rst);
 	if(status==0){
 		printf("ERROR: writing the result (%d)\n", status);
 		return 1;
 	}

 	printf("\nDONE!\n");
 	return 0;
}
