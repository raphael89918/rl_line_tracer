/**
 *  Console IO implementation under Linux.
**/

#ifdef __unix__

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <sys/select.h>
#include <sys/types.h>

int GetCharacter(int enableEcho);

int getch(void)
{
    return GetCharacter(0);
}

int getche(void)
{
    return GetCharacter(1);
}

int GetCharacter(int enableEcho)
{
	int iResult;

	char tmp = EOF;
	struct termios tBak, tSet;

	fd_set readFD;

	// Backup current terminal input interface
	iResult = tcgetattr(STDIN_FILENO, &tBak);
	if(iResult < 0)
	{
		goto RET;
	}

	// Set new terminal input interface
	tSet = tBak;
	if(enableEcho > 0)
	{
		tSet.c_lflag = tSet.c_lflag & ~(ICANON);
	}
	else
	{
		tSet.c_lflag = tSet.c_lflag & ~(ICANON | ECHO);
	}

	iResult = tcsetattr(STDIN_FILENO, TCSANOW, &tSet);
	if(iResult < 0)
	{
		goto RET;
	}

	// Processing fd set
	FD_ZERO(&readFD);
	FD_SET(STDIN_FILENO, &readFD);
	iResult = select(STDIN_FILENO + 1, &readFD, NULL, NULL, NULL);
	if(iResult < 0)
	{
		tmp = EOF;
		goto RET;
	}

	// Get a character
	tmp = getchar();

	// Restore setting
	tcsetattr(STDIN_FILENO, TCSANOW, &tBak);

RET:
    return (int)tmp;
}

int kbhit(void)
{
	int iResult;
	struct termios tBak, tSet;
	int retValue = 0;
	
	fd_set readFD;
	struct timeval timeout;

	// Backup current terminal input interface
	iResult = tcgetattr(STDIN_FILENO, &tBak);
	if(iResult < 0)
	{
		goto RET;
	}

	// Set new terminal input interface
	tSet = tBak;
	tSet.c_lflag = tSet.c_lflag & ~(ICANON | ECHO);
	iResult = tcsetattr(STDIN_FILENO, TCSANOW, &tSet);
	if(iResult < 0)
	{
		goto RET;
	}

	// Set timeout
	timeout.tv_sec = 0;
	timeout.tv_usec = 0;

	// Processing fd set
	FD_ZERO(&readFD);
	FD_SET(STDIN_FILENO, &readFD);
	iResult = select(STDIN_FILENO + 1, &readFD, NULL, NULL, &timeout);
	if(iResult > 0)
	{
		retValue = 1;
	}

	// Restore terminal mode
	tcsetattr(STDIN_FILENO, TCSANOW, &tBak);

RET:
    return retValue;
}

#endif
