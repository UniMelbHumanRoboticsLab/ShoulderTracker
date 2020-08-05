//--------------------------------------------------------------------------
//
//    Copyright (C) 2015-2016, 2020 Vincent Crocher
//    The University of Melbourne
//
//	  This file is part of ShoulderTrackingIMU.
//
//
//---------------------------------------------------------------------------
#include <getopt.h>
#include "MainWindow.h"


#define _WIN32_WINNT 0x0400
#pragma comment( lib, "user32.lib" )

#include <windows.h>
#include <stdio.h>


int main (int argc, char ** argv)
{
    mode_type InitMode;
    bool Plotting=false;

    //POSIX command line arguments: PLOTTING mode and Static/Dynamic
    int OptionChar;             //Option character
    bool mode_spec=false;
    while (1)
    {
        static struct option long_options[] =
        {
            //Supported options
            {"mode", required_argument,       0, 'm'},
            {"plotting",  no_argument,       0, 'p'},
            //{"output",    required_argument, 0, 'o'},
            {0, 0, 0, 0}
        };
        // getopt_long stores the option index here.
        int option_index = 0;

        OptionChar = getopt_long (argc, argv, "pm:", long_options, &option_index);

        // Detect the end of the options
        if (OptionChar == -1)
            break;

         switch (OptionChar)
         {
             case 0:
                break;

             //Mode argument
             case 'm':
             case 'M':
                //Get mode:
                switch(optarg[0])
                {
                    case 's':
                    case 'S':
                        printf("STATIC mode\n");
                        InitMode=Static;
                        mode_spec=true;
                        break;

                    case 'd':
                    case 'D':
                        printf("DYNAMIC mode\n");
                        InitMode=Dynamic;
                        mode_spec=true;
                        break;

                    default:
                        fprintf(stderr, "Error: Please provide a valid mode (-m S: Static or -m D: Dynamic).\nUsage example:\t %s -m S\n\n", argv[0]);
                        exit(0);
                }
                break;

             //Get plotting option
             case 'p':
             case 'P':
                Plotting=true;
                printf("Plotting ON.\n");
                break;

             default:
                fprintf(stderr, "Error: Please provide a valid mode (-m S: Static or -m D: Dynamic).\nUsage example:\t %s -m S\n\n", argv[0]);
                exit(0);
         }
    }
    if(argc<2 || (!mode_spec && !Plotting))
    {
        fprintf(stderr, "Error: Please provide a valid mode (-m S: Static or -m D: Dynamic).\nUsage example:\t %s -m S\n\n", argv[0]);
        exit(0);
    }

    HANDLE hThread;
    DWORD dwThread;

    hThread = CreateThread(NULL,NULL,(LPTHREAD_START_ROUTINE)
        MyMouseLogger, (LPVOID) argv[0], NULL, &dwThread);


    MainWindow *mw=new MainWindow(InitMode, Plotting);
    Fl::run();

    delete mw;

    return 0;
}
