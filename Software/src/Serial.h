//--------------------------------------------------------------------------
//
//    Copyright (C) 2015-2016 Vincent Crocher
//    The University of Melbourne
//
//	  This file is part of ShoulderTrackingIMU.
//
//
//---------------------------------------------------------------------------
#ifndef SERIAL_H
#define SERIAL_H

#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <FL/Fl.H>
#include <FL/fl_ask.H>

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

        int Read(char *mode, char *state, float *device_time, float *vals);
        bool CheckDevice();
        bool SetState(bool play);
        bool SetMode(mode_type mode);

        bool GetConnected() { return Connected; }
        void SetConnected(bool val) { Connected = val; }

    protected:
    private:
        FILE * PortCom;
        bool Connected;
};

#endif // SERIAL_H
