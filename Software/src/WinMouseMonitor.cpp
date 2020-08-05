#include "WinMouseMonitor.h"

HHOOK hMouseHook;
int MouseActivity;
int MousePosition[2];

__declspec(dllexport) LRESULT CALLBACK MouseEvent (int nCode, WPARAM wParam, LPARAM lParam)
{
    MOUSEHOOKSTRUCT * pMouseStruct = (MOUSEHOOKSTRUCT *)lParam;
    if (pMouseStruct != NULL)
    {
        //printf("Mouse position X = %d  Mouse Position Y = %d %d\n", pMouseStruct->pt.x,pMouseStruct->pt.y,MouseActivity);
        MouseActivity++;
        MousePosition[0]=pMouseStruct->pt.x;
        MousePosition[1]=pMouseStruct->pt.y;
    }
    return CallNextHookEx(hMouseHook,
        nCode,wParam,lParam);
}

void MessageLoop()
{
    MSG message;
    while (GetMessage(&message,NULL,0,0)) {
        TranslateMessage( &message );
        DispatchMessage( &message );
    }
}

DWORD WINAPI MyMouseLogger(LPVOID lpParm)
{
    HINSTANCE hInstance = GetModuleHandle(NULL);
    if (!hInstance) hInstance = LoadLibrary((LPCSTR) lpParm);
    if (!hInstance) return 1;

    MouseActivity=0;

    hMouseHook = SetWindowsHookEx (
        WH_MOUSE_LL,
        (HOOKPROC) MouseEvent,
        hInstance,
        NULL
        );
    MessageLoop();
    UnhookWindowsHookEx(hMouseHook);
    return 0;
}
