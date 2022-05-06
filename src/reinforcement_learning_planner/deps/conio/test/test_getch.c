#include <stdio.h>
#include <stdlib.h>

#include "conio.h"

int main()
{
    int iResult = 0;
	
	// Test getch()
	fprintf(stderr, "Test getch()\n");
	fprintf(stderr, "Input a character: ");
	iResult = getch();
	fprintf(stderr, "\ngetch() = %c\n\n", iResult);

    return 0;
}
