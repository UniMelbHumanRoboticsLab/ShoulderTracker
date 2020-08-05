//--------------------------------------------------------------------------
//
//    Copyright (C) 2015-2016 Vincent Crocher
//    The University of Melbourne
//
//	  This file is part of ShoulderTrackingIMU.
//
//
//---------------------------------------------------------------------------
#include "Serial.h"



Serial::Serial(bool quiet)
{
    Connected=false;
    PortCom=NULL;
    Connect(quiet);
}

Serial::~Serial()
{
    Disconnect();
}


bool Serial::Connect(bool quiet)
{
    if(!Connected)
    {
        //Try nickname (symlink) first
        PortCom=fopen("/dev/arduino_leonardo", "a+");
        if(!PortCom)
        {
            PortCom=fopen("/dev/ttyACM0", "a+");
            if(!PortCom)
            {
                PortCom=fopen("/dev/ttyACM1", "a+");
                if(!PortCom)
                {
                    PortCom=fopen("/dev/ttyACM2", "a+");
                    if(!PortCom)
                    {
                        PortCom=fopen("/dev/ttyACM3", "a+");
                        if(!PortCom)
                        {
                            PortCom=fopen("/dev/ttyACM4", "a+");
                            if(!PortCom)
                            {
                                printf("Unable to open the port (/dev/ttyACM0 to /dev/ttyACM4 neither /dev/arduino_leonardo1).\n");
                                if(!quiet)
                                    fl_alert("Shoulder Tracker not detected.\n\n Is the device ON?\n Have you plugged the USB dongle?\n");
                                Connected=false;
                            }
                            else
                            {
                                Connected=true;
                            }
                        }
                        else
                        {
                            Connected=true;
                        }
                    }
                    else
                    {
                        Connected=true;
                    }
                }
                else
                {
                    Connected=true;
                }
            }
            else
            {
                Connected=true;
            }
        }
        else
        {
            Connected=true;
        }
    }

    //Then check device
    if(Connected)
    {
        printf("Connected...\n", PortCom+1);
        usleep(1000);

        //Check if it's a shoulder tracker
        printf("Is device a shoulder tracker?");
        if(CheckDevice())
        {
            printf("\t YES.\n");
        }
        else
        {
            Disconnect();
            printf("\t NO.\n");
        }
    }

    return Connected;
}

//!Ask device to stop transmitting and close connection
void Serial::Disconnect()
{
    if(Connected)
    {
        SetState(false);
        fclose(PortCom);
    }

    Connected=false;
}


int Serial::SendChar(unsigned char c)
{
    if(Connected)
    {
        fprintf(PortCom, "%c\n", c);
        return 0;
    }

    return -1;
}

int Serial::SendChars(const char *c, int nb_vals)
{
    if(Connected)
    {
        unsigned char mess[255];
        for(int i=0; i<fmin(nb_vals,255); i++)
            mess[i]=c[i];

        fprintf(PortCom, "%s\n", mess);
        return 0;
    }

    return -1;
}


int Serial::Read(char *mode, char *state, float *device_time, float *vals)
{
    if(Connected)
    {
        if(fscanf(PortCom, "%c%c%f,%f,%f,%f,%f", mode, state, device_time, &vals[0], &vals[1], &vals[2], &vals[3])==7)
        {
            //Flush buffer
            char r;
            while(r!=EOF)
                r = fgetc(PortCom);

            //Check that values looks correct
            if( (*mode=='D' || *mode=='S') && (*state=='P' || *state=='R') )
                return 0;
            else
                return -3;
        }
        else
            return -2;
    }
    else //Not connected
    {
        //Try to connect...
        if(Connect(true))
        {
            //Read again if finally connected
            return Read(mode, state, device_time, vals);
        }
        else
        {
            //error otherwise
            return -1;
        }
    }
}


//!Check if the connected device is a shoulder tracker
//! by sending query command
bool Serial::CheckDevice()
{
    if(Connected)
    {
        //Send query (CDQ)
        if(SendChars("CDQ", 3)==0)
        {
            printf("checking...\n");
            usleep(200000);//Give time to Arduino to reply: NEEDED

            //Get reply: should be "OKST"
            unsigned char reply[5]={'\0','\0','\0','\0','\0'};
            if(fscanf(PortCom, "%4c", reply)==1);
            {
                printf("%s\n", reply);
                //Flush buffer
                char r;
                while(r!=EOF)
                    r = fgetc(PortCom);

                //Check reply
                if(strcmp((char*)reply, "OKST")==0)
                    return true;
            }
        }
    }

    return false;
}



//!Ask for Play (SetState(true)) or Pause (SetState(false))
//!\return true if success
bool Serial::SetState(bool play)
{
    if(Connected)
    {
        //Flush buffer
        char r;
        while(r!=EOF)
            r = fgetc(PortCom);

        //Send running (CDR) or pause (CDP)
        char cmd[4], expected_reply[4];
        if(play)
        {
            sprintf(cmd, "CDR");
            sprintf(expected_reply, "OK");
        }
        else
        {
            sprintf(cmd, "CDP");
            sprintf(expected_reply, "OK");
        }
        //Send it
        if(SendChars(cmd, 3)==0)
        {
            usleep(100000);//Give time to Arduino to reply: NEEDED

            //Get reply: should be "OKxP" or "OKxR"
            unsigned char reply[10];
            if(fscanf(PortCom, "%9c", reply)==1);
            {
                //Flush buffer
                char r;
                while(r!=EOF)
                    r = fgetc(PortCom);

                printf("-%s-\n", reply);
                //Check reply
                if( strstr((char*)reply, expected_reply) != NULL )
                    return true;
            }
        }
    }
    else
    {
        //Try to connect and retry
        if(Connect(true))
            return SetState(play);
    }

    return false;
}


bool Serial::SetMode(mode_type mode)
{
    if(Connected)
    {
        //Flush buffer
        char r;
        while(r!=EOF)
            r = fgetc(PortCom);

        //Send running (CDR) or pause (CDP)
        char cmd[4], expected_reply[4];
        if(mode==Static)
        {
            sprintf(cmd, "CDS");
            sprintf(expected_reply, "OK");
        }
        else
        {
            sprintf(cmd, "CDD");
            sprintf(expected_reply, "OK");
        }
        //Send it
        if(SendChars(cmd, 3)==0)
        {
            usleep(100000);//Give time to Arduino to reply: NEEDED

            //Get reply: should be "OKxP" or "OKxR"
            unsigned char reply[10];
            if(fscanf(PortCom, "%9c", reply)==1);
            {
                //Flush buffer
                char r;
                while(r!=EOF)
                    r = fgetc(PortCom);

                printf("-%s-\n", reply);
                //Check reply
                if( strstr((char*)reply, expected_reply) != NULL )
                    return true;
            }
        }
    }
    else
    {
        //Try to connect and retry
        if(Connect(true))
            return SetMode(mode);
    }

    return false;
}
