#pragma once

#include"TSP.h"
#include <stdio.h>
#include <string.h>
//Main function calling different methods to solve TSP
//Print output into .sol and .trace file

using namespace std;

extern ofstream output_sol,output_trace;

int callTSP();

// int main(int argc, char* argv[]){
// 	callTSP();
// 	return 0;
// }