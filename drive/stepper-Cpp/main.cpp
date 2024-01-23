#define NOMINMAX
#include <windows.h>

#include <iostream>

#include <unistd.h>


#include "NEMA34new.h"



int main()
{

	NEMA34 m1("\\\\.\\COM26");

	m1.WRI(22, 80000000);	//set velocity

	m1.WRI(21, 80000);		//set accell

	m1.PVC(21,0,0,0);		//start motor



	usleep(3000000);		//wait 3 sec


	m1.STP(8000);			//stop motor

}
