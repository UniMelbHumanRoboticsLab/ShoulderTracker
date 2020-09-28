//--------------------------------------------------------------------------
//
//    Copyright (C) 2015-2016, 2020 Vincent Crocher
//    The University of Melbourne
//
//	  This file is part of ShoulderTrackingIMU.
//
//
//---------------------------------------------------------------------------
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <stdio.h>
#include <sys/time.h> //Just to measure time
#include <time.h>
#include <FL/Fl.H>
#include <FL/Fl_Tabs.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_Pack.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Radio_Round_Button.H>
#include <FL/Fl_File_Input.H>
#include <FL/Fl_Double_Window.H>
#include <FL/filename.H>
#include <FL/Fl_Preferences.H> //To get user data path

#include "Fl_TimerSimple.H"

#ifdef WINDOWS
    #include "SerialWin.h"
    #include "windows.h"
    #include "Mmsystem.h"
    #pragma comment(lib,"winmm.lib")
#else
    #include "Serial.h"
#endif
#include "Plots.h"
#include "WinMouseMonitor.h"
#include "GameWindow.h"


void UpdateValues_cb(void * param);
void CheckMouseActivity_cb(void * param);
void AutoConnectTimer_cb(void * param);
void PlayPauseButton_cb(Fl_Widget * widget, void * param);
void ClearButton_cb(Fl_Widget * widget, void * param);
void ModeGroup_cb(Fl_Widget * widget, void * param);
void FreqButton_cb(Fl_Widget * widget, void * param);
void MoveButton_cb(Fl_Widget * widget, void * param);
void Quit_cb(Fl_Widget * widget, void * param);
void SetInterventionButton_cb(Fl_Widget * widget, void * param);



class MainWindow
{
    public:
        MainWindow(mode_type init_mode, bool plotting);
        ~MainWindow();

        void GenerateFilename();
        bool ReadInterventionState();
        void SetToIntervention();
        void SetToBaseline();

        friend void UpdateValues_cb(void * param);
        friend void CheckMouseActivity_cb(void * param);
        friend void AutoConnectTimer_cb(void * param);
        friend void PlayPauseButton_cb(Fl_Widget * widget, void * param);
        friend void ClearButton_cb(Fl_Widget * widget, void * param);
        friend void ModeGroup_cb(Fl_Widget * widget, void * param);
        friend void FreqButton_cb(Fl_Widget * widget, void * param);
        friend void MoveButton_cb(Fl_Widget * widget, void * param);
        friend void Quit_cb(Fl_Widget * widget, void * param);
        friend void SetInterventionButton_cb(Fl_Widget * widget, void * param);

    public:
        Fl_Double_Window *Window, *MinWindow;
        GameWindow *AssessGameWindow;

        Fl_Box *StatusBar;
        Plots * Plot;

        Fl_Button * PlayPauseButton, * ClearButton;
        Fl_Pack *ModeGroup;
        Fl_Radio_Round_Button *StaticButton, *DynamicButton;
        Fl_File_Input * FilenameInput;

        Fl_Box *TitleBox;
        Fl_Box *OnOffBox;
        Fl_TimerSimple *TimeLabel;
        Fl_Button *QuitButton, *SetInterventionButton;

        Serial *SerialCom;
        FILE *logFile;
        char Filename[1024], logPath[FL_PATH_MAX];
        Fl_Preferences *Preferences;
        bool Play, MouseActive;
        int NbMissedUpdates, NbMissedConnections;
        char Mode, State;
        mode_type InitMode;
        bool WasConnected;
        bool Intervention; //! When true, feedback will be provided during the session, otherwise (baseline period) no feedback is provided, device stays in test mode.
};

#endif // MAINWINDOW_H
