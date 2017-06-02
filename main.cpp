#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <time.h>
#include <string> 
#include <map>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <time.h>

#include "Manager.h"


using namespace std;

ifstream infile;
ofstream outfile;


int main(int argc, const char **argv)
{
	clock_t t_first,t_last;
	t_first = clock();
	
	Manager Mgr(argv[1],argv[2]);
	

	t_last = clock();
	float second = ((float)t_last-(float)t_first)/CLOCKS_PER_SEC;
	cout << "\ntimes: "<<second << "s"<<"\nEXIT! \n" <<endl;
}




/*
command:
./main.exe cases/case1 test
*/
