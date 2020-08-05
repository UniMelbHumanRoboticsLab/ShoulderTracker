//--------------------------------------------------------------------------
//
//    Copyright (C) 2015-2016 Vincent Crocher
//    The University of Melbourne
//
//	  This file is part of ShoulderTrackingIMU.
//
//
//---------------------------------------------------------------------------
#include "Plots.h"

Plots::Plots(int xx, int yy, int ww, int hh, const char *label): Fl_Group(xx, yy, ww, hh, label)
{
    const char plot_label[4][255]={"Value 1", "Value 2", "Threshold 1", "Threshold 2" };
    PlotColors[0]=FL_DARK_RED;
    PlotColors[1]=FL_DARK_GREEN;
    for(int i=0; i<2; i++)
    {
        //sprintf(plot_label, "Joint %d", i+1);
        SubPlots[i]=new Fl_Chart(xx, yy+i*((int)(hh/2.)), ww-100, (int)(hh/2.)-20, "");
        SubPlots[i]->type(FL_LINE_CHART);
        SubPlots[i]->box(FL_EMBOSSED_BOX);
        SubPlots[i]->copy_label(plot_label[i]);
        SubPlots[i]->labelcolor(PlotColors[i]);
        SubPlots[i]->color(FL_WHITE);
        SubPlots[i]->maxsize(1000);
        SubPlots[i]->autosize(0);
    }
    for(int i=0; i<2; i++)
    {
        //sprintf(plot_label, "Joint %d", i+1);
        SubPlots[i+2]=new Fl_Chart(xx+ww-100, yy+i*((int)(hh/2.)), 100, (int)(hh/2.)-20, "");
        SubPlots[i+2]->type(FL_LINE_CHART);
        SubPlots[i+2]->box(FL_EMBOSSED_BOX);
        SubPlots[i+2]->copy_label(plot_label[i+2]);
        SubPlots[i+2]->labelcolor(PlotColors[i]);
        SubPlots[i+2]->color(FL_WHITE);
        SubPlots[i+2]->maxsize(1000);
        SubPlots[i+2]->autosize(0);
    }
    /*SubPlots[0]->bounds(0, 1);
    SubPlots[1]->bounds(0, 2.);*/
    end();
}

Plots::~Plots()
{
    //dtor
}

void Plots::AddValues(float *vals)
{
    for(int j=0; j<2; j++)
    {
        SubPlots[j]->add(vals[j], 0, PlotColors[j]); //Current value
        SubPlots[j+2]->add(vals[j+2], 0, PlotColors[j]);//Threshold
    }
    //printf("%f %f %f %f\n", vals[0], vals[1], vals[2], vals[3]);
}
