//--------------------------------------------------------------------------
//
//    Copyright (C) 2015 Vincent Crocher
//    The University of Melbourne
//
//	  This file is part of JointHeatMap.
//
//
//---------------------------------------------------------------------------
#include "Data.h"


void ISBtoTR(float q[5], float MatS[16], float MatE[16])
{
    //Trunk->Shoulder transformation (ISB is euler YXY)
    float cy0=cos(q[0]); float cx=cos(q[1]); float cy1=cos(q[2]);
	float sy0=sin(q[0]); float sx=sin(q[1]); float sy1=sin(q[2]);
    //Column 1
    MatS[0]=cy0*cy1-cx*sy0*sy1;
    MatS[1]=sx*sy1;
    MatS[2]=-cy1*sy0-cx*cy0*sy1;
    MatS[3]=0;
    //Column 2
    MatS[4]=sx*sy0;
    MatS[5]=cx;
    MatS[6]=sx*cy0;
    MatS[7]=0;
    //Column 3
    MatS[8]=cx*cy1*sy0+cy0*sy1;
    MatS[9]=-sx*cy1;
    MatS[10]=cx*cy0*cy1-sy0*sy1;
    MatS[11]=0;
    //Column 4 (translation)
    MatS[12]=0;
    MatS[13]=0;
    MatS[14]=0;
    MatS[15]=1;

    //Shoulder->Elbow transformation (NOT USED)
    //Column 1
    MatE[0]=1;
    MatE[1]=0;
    MatE[2]=0;
    MatE[3]=0;
    //Column 2
    MatE[4]=0;
    MatE[5]=1;
    MatE[6]=0;
    MatE[7]=0;
    //Column 3
    MatE[8]=0;
    MatE[9]=0;
    MatE[10]=1;
    MatE[11]=0;
    //Column 4 (translation)
    MatE[12]=0;
    MatE[13]=0;
    MatE[14]=0;
    MatE[15]=1;

}

void Differenciation(std::vector<LimbPosture>* post, std::vector<LimbPosture> *vel_postures)
{
    std::vector<LimbPosture> postures=(*post);

    //First point
    LimbPosture velpost;
    velpost.t=postures[0].t;
    velpost.weight=postures[0].weight;
    for(int j=0; j<5; j++)
        velpost.q[j]=(postures[1].q[j]-postures[0].q[j])/(postures[1].t-postures[0].t);
    vel_postures->push_back(velpost);

    //For each static posture (except extremes)
    for(int i=1; i<postures.size()-1; i++)
    {
        //Create a new "velocity" posture
        LimbPosture velpost;
        velpost.t=postures[i].t;
        velpost.weight=postures[i].weight;
        //For each angle compute differenciation
        for(int j=0; j<5; j++)
        {
            velpost.q[j]=(postures[i+1].q[j]-postures[i-1].q[j])/(postures[i+1].t-postures[i-1].t);
        }
        //Add it to the velocity postures
        vel_postures->push_back(velpost);
    }

    //Last point
    velpost.t=postures[postures.size()-1].t;
    velpost.weight=postures[postures.size()-1].weight;
    for(int j=0; j<5; j++)
        velpost.q[j]=(postures[postures.size()-1].q[j]-postures[postures.size()-1-1].q[j])/(postures[postures.size()-1].t-postures[postures.size()-1-1].t);
    vel_postures->push_back(velpost);
}



Data::Data(const char filename[1024])
{
    sprintf(Filename, "%s", filename);

    LoadISBFile();
    //Do an histogram analysis on the postures
    ISBFrequencyAnalysisWElbowAndProno();

    //Compute joint velocities
    Differenciation(&Postures, &Velocities);

    //LoadHeatMapFile();
}

Data::~Data()
{
    //dtor
}


//Load csv file containing a list of joint angles (ISB)
void Data::LoadISBFile()
{
    FILE * dataFile=fopen(Filename, "r");
    if(dataFile)
    {
        float q[5];
        int t;
        char crap[1024];
        //Scanf is supposed to get 6 values total
        while( fscanf(dataFile, "%d,%f,%f,%f,%f,%f", &t, &q[0], &q[1], &q[2], &q[3], &q[4]) !=6 )
            fscanf(dataFile, "%s", crap);

        //Now get actual data
        while( fscanf(dataFile, "%d,%f,%f,%f,%f,%f", &t, &q[0], &q[1], &q[2], &q[3], &q[4]) == 6 )
        {
            LimbPosture post;
            post.t=t;
            post.weight=1.0; //default weigt
            for(int i=0; i<5; i++)
                post.q[i]=q[i];
            Postures.push_back(post);
        }

        fclose(dataFile);
    }
    else
    {
        printf("Error opening: %s\n", Filename);
    }
}


//Load a csv file containing a list of three quaternions orientation
void Data::LoadSensorsFile()
{
    FILE * dataFile=fopen(Filename, "r");
    if(dataFile)
    {
        float q1[4], q2[4], q3[4];
        int t;
        char crap[1024];
        //Scanf is supposed to get 13 values total: first skip the header lines
        while( fscanf(dataFile, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &t, &q1[0], &q1[1], &q1[2], &q1[3], &q2[0], &q2[1], &q2[2], &q2[3], &q3[0], &q3[1], &q3[2], &q3[3]) != 13 )
            fscanf(dataFile, "%s", crap);

        //Now get actual data
        while( fscanf(dataFile, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &t, &q1[0], &q1[1], &q1[2], &q1[3], &q2[0], &q2[1], &q2[2], &q2[3], &q3[0], &q3[1], &q3[2], &q3[3]) == 13 )
        {
            QuatPosture post;
            post.t=t;
            for(int i=0; i<4; i++)
            {
                post.q1[i]=q1[i];
                post.q2[i]=q2[i];
                post.q3[i]=q3[i];
            }
            QuaternionsPostures.push_back(post);
        }

        fclose(dataFile);
    }
    else
    {
        printf("Error opening: %s\n", Filename);
    }
}

//Load csv file containing a list of joint angles (ISB)
void Data::LoadHeatMapFile()
{
    FILE * dataFile=fopen(Filename, "r");
    if(dataFile)
    {
        float q[5], w1, w2;
        char crap[1024];
        //Scanf is supposed to get 6 values total
        while( fscanf(dataFile, "%f,%f,%f,%f,%f,%f,%f", &q[0], &q[1], &q[2], &q[3], &q[4], &w1, &w2) != 7 )
            fscanf(dataFile, "%s", crap);

        //Now get actual data
        while( fscanf(dataFile, "%f,%f,%f,%f,%f,%f,%f", &q[0], &q[1], &q[2], &q[3], &q[4], &w1, &w2) == 7 )
        {
            LimbPosture post;
            post.t=0;
            post.weight=w1/100.;
            for(int i=0; i<5; i++)
                post.q[i]=q[i];
            Postures.push_back(post);
        }

        fclose(dataFile);
    }
    else
    {
        printf("Error opening: %s\n", Filename);
    }
}


void Data::ISBFrequencyAnalysis()
{
    //A Mat containing the shoulder joint angles: 3 channels (one per angle) and NbSamplesx1 size
    Mat ShoulderAngles = Mat::zeros(Postures.size(), 1, CV_32FC3);

    //Fill with joint angles
    for(int i=0; i<Postures.size(); i++)
    {
        ShoulderAngles.at<Vec3f>(i, 0)[0] = Postures[i].q[0];
        ShoulderAngles.at<Vec3f>(i, 0)[1] = Postures[i].q[1];
        ShoulderAngles.at<Vec3f>(i, 0)[2] = Postures[i].q[2];
    }

    //Histogram on shoulder angles
    Mat ShoulderHistCount;
    int nb_bins=10;
    const int channelsList[]={0,1,2};
    int histSize[] = {nb_bins, nb_bins, nb_bins};
    float range[]= {-M_PI,M_PI};
    const float* histRanges[] = {range, range, range};
    calcHist(&ShoulderAngles, 1, channelsList, Mat(), ShoulderHistCount, 3, histSize, histRanges, true, false);

    //Find Shoulder angles values of the middle of each bin
    Mat ShoulderHistMid(3, histSize, CV_32FC3);
    float RangeWidth[3], BinWidth[3];
    for(int i=0; i<3; i++)
    {
        RangeWidth[i]=histRanges[i][1]-histRanges[i][0];
        BinWidth[i]=RangeWidth[i]/(float)nb_bins;
    }
    for(int i=0; i<nb_bins; i++)
        for(int j=0; j<nb_bins; j++)
            for(int k=0; k<nb_bins; k++)
            {
                ShoulderHistMid.at<Vec3f>(i, j, k)[0]=histRanges[0][0]+BinWidth[0]*i+BinWidth[0]/2.;
                ShoulderHistMid.at<Vec3f>(i, j, k)[1]=histRanges[1][0]+BinWidth[1]*j+BinWidth[1]/2.;
                ShoulderHistMid.at<Vec3f>(i, j, k)[2]=histRanges[2][0]+BinWidth[2]*k+BinWidth[2]/2.;
            }

    //Weight is 0-1 so need max weight
    double maxCount;
    minMaxIdx(ShoulderHistCount, 0, &maxCount, 0, 0);

    //Clear existing posture array first
    FrequencyPostures.clear();

    //Push everything in the vector
    for(int i=0; i<nb_bins; i++)
        for(int j=0; j<nb_bins; j++)
            for(int k=0; k<nb_bins; k++)
            {
                LimbPosture post;
                post.t=0;
                post.weight=(float)ShoulderHistCount.at<float>(i, j, k)/(float)(maxCount);
                if(post.weight>0)
                {
                    post.q[0]=ShoulderHistMid.at<Vec3f>(i, j, k)[0];
                    post.q[1]=ShoulderHistMid.at<Vec3f>(i, j, k)[1];
                    post.q[2]=ShoulderHistMid.at<Vec3f>(i, j, k)[2];
                    post.q[3]=1.5;
                    post.q[4]=0;
                    FrequencyPostures.push_back(post);
                }
            }
    printf("%d Postures with a frequency >0\n", FrequencyPostures.size());
}

void Data::ISBFrequencyAnalysisWElbow()
{
    //A Mat containing the shoulder joint angles: 4 channels (one per angle) and NbSamplesx1 size
    Mat ShoulderAngles = Mat::zeros(Postures.size(), 1, CV_32FC4);

    //Fill with joint angles
    for(int i=0; i<Postures.size(); i++)
    {
        ShoulderAngles.at<Vec3f>(i, 0)[0] = Postures[i].q[0];
        ShoulderAngles.at<Vec3f>(i, 0)[1] = Postures[i].q[1];
        ShoulderAngles.at<Vec3f>(i, 0)[2] = Postures[i].q[2];
        ShoulderAngles.at<Vec3f>(i, 0)[3] = Postures[i].q[3];
    }

    //Histogram on shoulder angles
    Mat ShoulderHistCount;
    int nb_bins=20;
    const int channelsList[]={0,1,2,3};
    int histSize[] = {nb_bins, nb_bins, nb_bins, nb_bins};
    float range[]= {-M_PI,M_PI};
    const float* histRanges[] = {range, range, range, range};
    calcHist(&ShoulderAngles, 1, channelsList, Mat(), ShoulderHistCount, 4, histSize, histRanges, true, false);

    //Find Shoulder angles values of the middle of each bin
    Mat ShoulderHistMid(4, histSize, CV_32FC3);
    float RangeWidth[4], BinWidth[4];
    for(int i=0; i<4; i++)
    {
        RangeWidth[i]=histRanges[i][1]-histRanges[i][0];
        BinWidth[i]=RangeWidth[i]/(float)nb_bins;
    }
    for(int i=0; i<nb_bins; i++)
        for(int j=0; j<nb_bins; j++)
            for(int k=0; k<nb_bins; k++)
                for(int l=0; l<nb_bins; l++)
                {
                    const int idx[]={i, j, k, l};
                    ShoulderHistMid.at<Vec4f>(idx)[0]=histRanges[0][0]+BinWidth[0]*i+BinWidth[0]/2.;
                    ShoulderHistMid.at<Vec4f>(idx)[1]=histRanges[1][0]+BinWidth[1]*j+BinWidth[1]/2.;
                    ShoulderHistMid.at<Vec4f>(idx)[2]=histRanges[2][0]+BinWidth[2]*k+BinWidth[2]/2.;
                    ShoulderHistMid.at<Vec4f>(idx)[3]=histRanges[3][0]+BinWidth[3]*l+BinWidth[3]/2.;
                }

    //Weight is 0-1 so need max weight
    double maxCount;
    minMaxIdx(ShoulderHistCount, 0, &maxCount, 0, 0);

    //Clear existing vector first
    FrequencyPostures.clear();

    //Push everything in the vector
    for(int i=0; i<nb_bins; i++)
        for(int j=0; j<nb_bins; j++)
            for(int k=0; k<nb_bins; k++)
                for(int l=0; l<nb_bins; l++)
                {
                    const int idx[]={i, j, k, l};
                    LimbPosture post;
                    post.t=0;
                    post.weight=(float)ShoulderHistCount.at<float>(idx)/(float)(maxCount);
                    if(post.weight>0)
                    {
                        post.q[0]=ShoulderHistMid.at<Vec4f>(idx)[0];
                        post.q[1]=ShoulderHistMid.at<Vec4f>(idx)[1];
                        post.q[2]=ShoulderHistMid.at<Vec4f>(idx)[2];
                        post.q[3]=ShoulderHistMid.at<Vec4f>(idx)[3];
                        post.q[4]=0;
                        FrequencyPostures.push_back(post);
                    }
                }
    printf("%d Postures with a frequency >0\n", FrequencyPostures.size());
}



void Data::ISBFrequencyAnalysisWElbowAndProno()
{
    //A Mat containing the shoulder joint angles: 4 channels (one per angle) and NbSamplesx1 size
    Mat ShoulderAngles = Mat::zeros(Postures.size(), 1, CV_32FC(5));

    //Fill with joint angles
    for(int i=0; i<Postures.size(); i++)
    {
        ShoulderAngles.at<Vec3f>(i, 0)[0] = Postures[i].q[0];
        ShoulderAngles.at<Vec3f>(i, 0)[1] = Postures[i].q[1];
        ShoulderAngles.at<Vec3f>(i, 0)[2] = Postures[i].q[2];
        ShoulderAngles.at<Vec3f>(i, 0)[3] = Postures[i].q[3];
        ShoulderAngles.at<Vec3f>(i, 0)[4] = Postures[i].q[4];
    }

    //Histogram on shoulder angles
    Mat ShoulderHistCount;
    float accuracy=10./180.*M_PI; //Width of each bin in radians
    const int channelsList[]={0,1,2,3,4};
    float range[5][2];
    range[0][0]=-M_PI;range[0][1]=M_PI; //Plane of elevation
    range[1][0]=-M_PI_2;range[1][1]=M_PI; //Elevation
    range[2][0]=-M_PI;range[2][1]=M_PI;//Int/Ext rotation
    range[3][0]=0;range[3][1]=M_PI; //Elbow felx/ext
    range[4][0]=-M_PI;range[4][1]=M_PI; //Pronosupination
    const float* histRanges[] = {range[0], range[1], range[2], range[3], range[4]};
    //Compute nb of bins required in each dimension to have the requested accuracy on the joint range
    int histSize[5], TotalNbBins=1;
    for(int i=0; i<5; i++)
    {
        histSize[i]=(int)round((range[i][1]-range[i][0])/accuracy);
        TotalNbBins*=histSize[i];
    }
    printf("%d bins in the histogram (total).\n", TotalNbBins);

    //Compute the histogram
    calcHist(&ShoulderAngles, 1, channelsList, Mat(), ShoulderHistCount, 5, histSize, histRanges, true, false);

    //Find Shoulder angles values of the middle of each bin
    Mat ShoulderHistMid(5, histSize, CV_32FC3);
    float RangeWidth[5], BinWidth[5];
    for(int i=0; i<5; i++)
    {
        RangeWidth[i]=histRanges[i][1]-histRanges[i][0];
        BinWidth[i]=RangeWidth[i]/(float)histSize[i];
    }
    for(int i=0; i<histSize[0]; i++)
        for(int j=0; j<histSize[1]; j++)
            for(int k=0; k<histSize[2]; k++)
                for(int l=0; l<histSize[3]; l++)
                    for(int m=0; m<histSize[4]; m++)
                    {
                        Fl::check();
                        const int idx[]={i, j, k, l, m};
                        ShoulderHistMid.at<Vec5f>(idx)[0]=histRanges[0][0]+BinWidth[0]*i+BinWidth[0]/2.;
                        ShoulderHistMid.at<Vec5f>(idx)[1]=histRanges[1][0]+BinWidth[1]*j+BinWidth[1]/2.;
                        ShoulderHistMid.at<Vec5f>(idx)[2]=histRanges[2][0]+BinWidth[2]*k+BinWidth[2]/2.;
                        ShoulderHistMid.at<Vec5f>(idx)[3]=histRanges[3][0]+BinWidth[3]*l+BinWidth[3]/2.;
                        ShoulderHistMid.at<Vec5f>(idx)[4]=histRanges[4][0]+BinWidth[4]*l+BinWidth[4]/2.;
                    }

    //Weight is 0-1 so need max weight
    double maxCount;
    minMaxIdx(ShoulderHistCount, 0, &maxCount, 0, 0);

    //Clear existing vector first
    FrequencyPostures.clear();

    //Push everything in the vector
    for(int i=0; i<histSize[0]; i++)
        for(int j=0; j<histSize[1]; j++)
            for(int k=0; k<histSize[2]; k++)
                for(int l=0; l<histSize[3]; l++)
                    for(int m=0; m<histSize[4]; m++)
                    {
                        Fl::check();
                        const int idx[]={i, j, k, l, m};
                        LimbPosture post;
                        post.t=0;
                        post.weight=(float)ShoulderHistCount.at<float>(idx)/(float)(maxCount);
                        if(post.weight>0)
                        {
                            post.q[0]=ShoulderHistMid.at<Vec5f>(idx)[0];
                            post.q[1]=ShoulderHistMid.at<Vec5f>(idx)[1];
                            post.q[2]=ShoulderHistMid.at<Vec5f>(idx)[2];
                            post.q[3]=ShoulderHistMid.at<Vec5f>(idx)[3];
                            post.q[4]=ShoulderHistMid.at<Vec5f>(idx)[4];
                            FrequencyPostures.push_back(post);
                        }
                    }
    printf("%d Postures with a frequency >0\n", FrequencyPostures.size());
}
