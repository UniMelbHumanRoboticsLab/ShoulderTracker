//--------------------------------------------------------------------------
//
//    Copyright (C) 2015 Vincent Crocher
//    The University of Melbourne
//
//	  This file is part of JointHeatMap.
//
//
//---------------------------------------------------------------------------
#ifndef DATA_H
#define DATA_H

#include <stdio.h>
#include <vector>
#include <math.h>
#include <FL/Fl.H> //For fl::wait();
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

//Not nativelly defined in OpenCV
typedef Vec<float, 5> Vec5f;


void ISBtoTR(float q[3], float MatS[16], float MatE[16]);


typedef struct LimbPosture
{
    int t;
    float q[5];
    float weight;
};

typedef struct QuatPosture
{
    int t;
    float q1[4], q2[4], q3[4];
};

class Data
{
    public:
        Data(const char filename[1024]);
        ~Data();

        void LoadISBFile();
        void LoadSensorsFile();
        void LoadHeatMapFile();


        void ISBFrequencyAnalysis();
        void ISBFrequencyAnalysisWElbow();
        void ISBFrequencyAnalysisWElbowAndProno();

        char Filename[1024];

        std::vector<LimbPosture> Postures;
        std::vector<LimbPosture> Velocities;
        std::vector<LimbPosture> FrequencyPostures;
        std::vector<QuatPosture> QuaternionsPostures;
};

#endif // DATA_H
