//--------------------------------------------------------------------------
//
//    Copyright (C) 2015-2016, 2020 Vincent Crocher
//    The University of Melbourne
//
//	  This file is part of ShoulderTrackingIMU.
//
//
//---------------------------------------------------------------------------
#include "SerialWin.h"



unsigned int Int16toInt(unsigned char LSB, unsigned char HSB)
{
    return LSB + HSB*256;
}

unsigned int Int32toInt(unsigned char LLSB, unsigned char LSB, unsigned char HSB, unsigned char HHSB)
{
    return  (((HHSB*256) + HSB)*256 + LSB)*256 + LLSB;
}



Serial::Serial(bool quiet)
{
    //Try any COM port...
    Connected=false;
    PortCom=0;
    Connect(quiet);
}

Serial::~Serial()
{
    Disconnect();
}


bool Serial::Connect(bool quiet)
{
    //Dont try again
    if(Connected)
        return true;

    //Try any COM port...
    int baud_rate=19200;
    for(PortCom=0; PortCom<16; PortCom++)
    {
        if(!RS232_OpenComport(PortCom, baud_rate, "8N1"))
        {
            Connected=true;
            printf("Connected on port COM%d.\n", PortCom+1);
            RS232_enableDTR(PortCom);
            RS232_enableRTS(PortCom);
            Sleep(1);

            //Check if it's a shoulder tracker
            printf("Is device on COM%d a shoulder tracker?", PortCom+1);
            if(CheckDevice())
            {
                printf("\t YES.\n");
                break;
            }
            else
            {
                RS232_CloseComport(PortCom);
                Connected=false;
                printf("\t NO.\n");
            }
        }
    }

    if(!Connected)
    {
        printf("Unable to open the port (COM1 - COM16).\n");
        if(!quiet)
            fl_alert("Shoulder Tracker not detected.\n\n Is the device ON?\n Have you plugged the USB dongle?\n");
        Connected=false;
    }

    return Connected;
}

//!Ask device to stop transmitting and close connection
void Serial::Disconnect()
{
    if(Connected)
    {
        SetState(false);
        RS232_CloseComport(PortCom);
    }

    Connected=false;
}

int Serial::SendChar(unsigned char c)
{

    if(Connected)
    {
        RS232_SendByte(PortCom, c);
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

        RS232_SendBuf(PortCom, mess, nb_vals);
        return 0;
    }

    return -1;
}


//!Read a data frame from the device
int Serial::Read(char *mode, char *state, float *device_time, float *vals, float *thresh)
{
    if(Connected)
    {
        int nb_bytes_expected=1+1+6+1+6+1+6+1+6+1+6+1+6+1+6+2;//Each float is 6 bytes from Arduino and ending by CRLF.
        unsigned char *buffer=new unsigned char[nb_bytes_expected];

        //Get first char of the sequence
        unsigned char startbyte=0;
        int i=0;
        while( startbyte!='D' && startbyte!='S' && i<2*nb_bytes_expected)
        {
            i++;
            RS232_PollComport(PortCom, &startbyte, 1);
            Sleep(1); //1ms
        }
        if(i>=2*nb_bytes_expected)
            return -4;

        *mode=startbyte;

        //Get full sequence (minus start byte)
        if(RS232_PollComport(PortCom, buffer, nb_bytes_expected-1)==nb_bytes_expected-1)
        {
            //printf("--%s--\n\n", buffer);
            //Parse received bytes
            if(sscanf((char *)buffer, "%c%f,%f,%f,%f,%f,%f,%f", state, device_time, &vals[0], &vals[1], &vals[2], &vals[3], &thresh[0], &thresh[1])!=8)
            {
                //RS232_flushRX(PortCom);
                delete[] buffer;
                return -2;
            }
            delete[] buffer;

            //Flush buffer
            RS232_flushRX(PortCom);

            //Check that values looks correct
            if( (*mode=='D' || *mode=='S') && (*state=='P' || *state=='R') )
                return 0;
            else
                return -3;
        }
        else //Wrong nb of bytes received
        {
            delete[] buffer;
            return -2;
        }
    }
    else //Not connected
    {
        //Try to connect...
        if(Connect(true))
        {
            //Read again if finally connected
            return Read(mode, state, device_time, vals, thresh);
        }
        else
        {
            //error otherwise
            return -1;
        }
    }
}

//!Read binary formatted data frame from the device
int Serial::ReadBinary(char *mode, char *state, float *device_time, float *vals, float *thresh)
{
    if(Connected)
    {
        int nb_bytes_expected=1+1+4+1+1+1+1+2+2+2;//Ending by CRLF.
        unsigned char *buffer=new unsigned char[nb_bytes_expected-1];

        //Get first char of the sequence
        unsigned char startbyte=0;
        int i=0;
        while( startbyte!='D' && startbyte!='S' && i<2*nb_bytes_expected)
        {
            i++;
            RS232_PollComport(PortCom, &startbyte, 1);
            Sleep(1); //1ms
        }
        if(i>=2*nb_bytes_expected)
            return -4;

        (*mode)=startbyte;

        //Get full sequence (minus start byte)
        if(RS232_PollComport(PortCom, buffer, nb_bytes_expected-1)==nb_bytes_expected-1)
        {
            //State: use as sanity check
            if(buffer[0]!='R' && buffer[0]!='T' && buffer[0]!='P')
                return -4;
            //State
            (*state)=buffer[0];
            //Time in s
            (*device_time) = (float) (Int32toInt(buffer[1], buffer[2], buffer[3], buffer[4]) /1000.); //LSB first
            //First angle deg
            vals[0] = buffer[5];
            //Second angle deg
            vals[1] = buffer[6];
            //First velocity
            vals[2] = (float)(buffer[7]/1000.);
            //Secondvelocity
            vals[3] = (float)(buffer[8]/1000.);
            //First threshold
            thresh[0] = (float)(Int16toInt(buffer[9], buffer[10])/100.);
            //Second threshold
            thresh[1] = (float)(Int16toInt(buffer[11], buffer[12])/100.);

            //CHECKSUM???
            //return -2;

            //Flush buffer
            RS232_flushRX(PortCom);

            //Check that values looks correct
            if( (*mode=='D' || *mode=='S') )
                return 0;
            else
                return -3;
        }
        else //Wrong nb of bytes received
        {
            delete[] buffer;
            return -2;
        }
    }
    else //Not connected
    {
        //Try to connect...
        if(Connect(true))
        {
            //Read again if finally connected
            return ReadBinary(mode, state, device_time, vals, thresh);
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
        //Flush buffer
        RS232_flushRX(PortCom);

        //Send query (CDQ)
        if(SendChars("CDQ", 3)==0)
        {
            Sleep(200);//Give time to Arduino to reply: NEEDED

            //Get reply: should be "OKST"
            unsigned char reply[5]={'\0','\0','\0','\0','\0'};
            if(RS232_PollComport(PortCom, reply, 4)==4);
            {
                //Flush buffer
                RS232_flushRX(PortCom);
                printf("reply: -%s-\n", reply);

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
        RS232_flushRX(PortCom);

        //Send running (CDR) or pause (CDP)
        char cmd[4], expected_reply[3];
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
            Sleep(100);//Give time to Arduino to reply: NEEDED

            //Get reply: should be "OKxP" or "OKxR"
            unsigned char reply[10]={'\0','\0','\0','\0','\0','\0','\0','\0','\0','\0'};
            if(RS232_PollComport(PortCom, reply, 10)>3);
            {
                //Flush buffer
                RS232_flushRX(PortCom);

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
//!Ask for Testing: recording but no feedback
//!\return true if success
bool Serial::SetTesting()
{
    if(Connected)
    {
        //Flush buffer
        RS232_flushRX(PortCom);

        //Send running (CDR) or pause (CDP)
        char cmd[4], expected_reply[3];
        sprintf(cmd, "CDT");
        sprintf(expected_reply, "OK");

        //Send it
        if(SendChars(cmd, 3)==0)
        {
            Sleep(100);//Give time to Arduino to reply: NEEDED

            //Get reply: should be "OKxP" or "OKxR"
            unsigned char reply[10]={'\0','\0','\0','\0','\0','\0','\0','\0','\0','\0'};
            if(RS232_PollComport(PortCom, reply, 10)>3);
            {
                //Flush buffer
                RS232_flushRX(PortCom);

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
            return SetTesting();
    }

    return false;
}



bool Serial::SetMode(mode_type mode)
{
    if(Connected)
    {
        //Flush buffer
        RS232_flushRX(PortCom);
        RS232_flushTX(PortCom);

        //Send running (CDR) or pause (CDP)
        char cmd[4], expected_reply[3];
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
            Sleep(100);//Give time to Arduino to reply: NEEDED

            //Get reply: should be "OKxP" or "OKxR"
            unsigned char reply[10]={'\0','\0','\0','\0','\0','\0','\0','\0','\0','\0'};;
            if(RS232_PollComport(PortCom, reply, 10)>3);
            {
                //Flush buffer
                RS232_flushRX(PortCom);

                printf("M-%s-\n", reply);
                //Check reply
                if( strstr((char*)reply, expected_reply) != NULL )
                {
                    //Wait Arduino re-initialization time when switching mode
                    Sleep(500);
                    return true;
                }
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
