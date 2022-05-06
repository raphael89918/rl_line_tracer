#include <stdio.h>
#include <stdlib.h>

#include "conio.h"

int main()
{
    int iResult = 0;
	
	// Test kbhit()
	fprintf(stderr, "Test kbhit()\n");
    while(iResult == 0)
    {
        iResult = kbhit();
        fprintf(stderr, "kbhit() = %d\r", iResult);
    }

    return 0;
}
