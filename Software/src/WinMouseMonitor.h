#ifndef WINMOUSEMONITOR_H_INCLUDED
#define WINMOUSEMONITOR_H_INCLUDED

#include <windows.h>
#include <stdio.h>

DWORD WINAPI MyMouseLogger(LPVOID lpParm);

extern int MouseActivity;
extern int MousePosition[2];


#endif // WINMOUSEMONITOR_H_INCLUDED
