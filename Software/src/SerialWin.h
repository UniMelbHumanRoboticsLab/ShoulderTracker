//--------------------------------------------------------------------------
//
//    Copyright (C) 2015-2016, 2020 Vincent Crocher
//    The University of Melbourne
//
//	  This file is part of ShoulderTrackingIMU.
//
//
//---------------------------------------------------------------------------
#ifndef SERIAL_H
#define SERIAL_H

#include <stdio.h>
#include <math.h>
#include <FL/Fl.H>
#include <FL/fl_ask.H>

#include "rs232.h"

enum mode_type {Static, Dynamic};

class Serial
{
    public:
        Serial(bool quiet=false);
        ~Serial();

        bool Connect(bool quiet=false);
        void Disconnect();

        int SendChar(unsigned char c);
        int SendChars(const char *c, int nb_vals);

        int Read(char *mode, char *state, float *device_time, float *vals, float *thresh);
        int ReadBinary(char *mode, char *state, float *device_time, float *vals, float *thresh);
        bool CheckDevice();
        bool SetState(bool play);
        void SetTesting(bool testingmode);
        bool SetMode(mode_type mode);

        bool GetConnected() { return Connected; }
        void SetConnected(bool val) { Connected = val; }

    private:
        int PortCom;
        bool Connected;
        bool TestingMode;
};

#endif // SERIAL_H
