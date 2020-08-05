//--------------------------------------------------------------------------
//
//    Copyright (C) 2015-2016 Vincent Crocher
//    The University of Melbourne
//
//	  This file is part of ShoulderTrackingIMU.
//
//
//---------------------------------------------------------------------------
#ifndef PLOTS_H
#define PLOTS_H

#include <FL/Fl_Group.H>
#include <FL/Fl_Chart.H>
#include <stdio.h>

class Plots : public Fl_Group
{
    public:
        Plots(int xx, int yy, int ww, int hh, const char *label);
        ~Plots();

        //void SetData(Data * data);
        void AddValues(float *val);

        void DrawDynamic(){for(int i=0; i<4; i++)SubPlots[i]->clear();SubPlots[0]->bounds(0, .05);SubPlots[1]->bounds(0, 2.);SubPlots[2]->bounds(0, .05);SubPlots[3]->bounds(0, 2.);} //Set plot bounds for dynamic mode
        void DrawStatic(){for(int i=0; i<4; i++){SubPlots[i]->clear();SubPlots[i]->bounds(0, 180);}} //Set plot bounds for static mode

    protected:
    private:
        Fl_Chart *SubPlots[4];
        Fl_Color PlotColors[4];
};

#endif // PLOTS_H
