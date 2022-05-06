#include <stdio.h>
#include <stdlib.h>

#include "conio.h"

int main()
{
    int iResult = 0;

	// Test getche()
	fprintf(stderr, "Test getche()\n");
	fprintf(stderr, "Input a character: ");
	iResult = getche();
	fprintf(stderr, "\ngetche() = %c\n", iResult);

    return 0;
}
