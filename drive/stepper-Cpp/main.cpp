#define NOMINMAX


#include <iostream>


#include "serialib.h"
#include "NEMA34new.h"



int main()
{

	NEMA34 m1("\\\\.\\COM26");

	m1.WRI(22, 80000000);	//set velocity

	m1.WRI(21, 80000);		//set accell

	m1.PVC(21,0,0,0);		//start motor


	//for Windows, use Sleep (uppercase)
	Sleep(3);		//wait 3 sec

	//for Linux, use sleep (lowercase)
	//sleep(3);		//wait 3 sec


	m1.STP(8000);			//stop motor

}
