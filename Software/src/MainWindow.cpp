//--------------------------------------------------------------------------
//
//    Copyright (C) 2015-2016, 2020 Vincent Crocher, Chenchen Liao
//    The University of Melbourne
//
//	  This file is part of ShoulderTrackingIMU.
//
//
//---------------------------------------------------------------------------
#include "MainWindow.h"


//!Timer cb that regularly polls values from the device
//! and update interface accordingly
void UpdateValues_cb(void * param)
{
    MainWindow *mw=(MainWindow*)param;

    //Get values
    char mode, state;
    float device_time;
    float vals[4], thresholds[2];
    double t_s;
    struct timeval t1;
    //if(mw->SerialCom->Read(&mode, &state, &device_time, vals, thresholds)>=0)
    if(mw->SerialCom->ReadBinary(&mode, &state, &device_time, vals, thresholds)>=0)
    {
        //Get time to log it
        gettimeofday(&t1, NULL);
        t_s = t1.tv_sec + t1.tv_usec / (1000.0*1000.0);

        //Reset nb of consecutive missed values
        mw->NbMissedUpdates=0;

        //Update status (mode and state)
        char status[100];

        //Status bar update
        if(mode!=mw->Mode || state!=mw->State)
        {
            if(mode=='D')
            {
                sprintf(status, "DYNAMIC\t");
                mw->Plot->DrawDynamic();
            }
            else
            {
                sprintf(status, "STATIC\t");
                mw->Plot->DrawStatic();
            }
            mw->State=state;
            if(state=='R' || state=='T')
                sprintf(status, "%sRUNNING", status);
            else
                sprintf(status, "%sPAUSE", status);
            mw->StatusBar->copy_label(status);
            mw->StatusBar->redraw();
        }

        //Plot (scale) update
        if(mode!=mw->Mode) //Has changed ?
        {
            mw->Mode=mode;
            if(mw->Mode=='D')
                mw->Plot->DrawDynamic();
            else
                mw->Plot->DrawStatic();
        }

        //Add to plot
        mw->Plot->AddValues(vals);
        mw->Plot->redraw();

        //Log
        char assessment_log_letter='G';//Set to A for assessment time, G during games
        if(mw->AssessGameWindow->visible())
            assessment_log_letter='A';
        if(mw->Play)
        {
            fprintf(mw->logFile, "%c,%c,%c,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d\n", assessment_log_letter, mw->Mode, mw->State, t_s, device_time, vals[0], vals[1], vals[2], vals[3], thresholds[0], thresholds[1], MousePosition[0], MousePosition[1]);
            //Provide audio feedback if required (not in assessment mode, not in baseline)
            if(!mw->SerialCom->IsTesting())
            {
                if(mw->Mode=='D')
                {
                    if(vals[3]>thresholds[1])//Use only rotational velocity
                    {
                       PlaySound(TEXT("slowdown.wav"), NULL, SND_FILENAME | SND_ASYNC | SND_NOSTOP);
                    }
                }
                else
                {
                    if(vals[0]>thresholds[0]||vals[1]>thresholds[1])
                    {
                        PlaySound(TEXT("reshape.wav"), NULL, SND_FILENAME | SND_ASYNC | SND_NOSTOP);
                    }
                }
            }
            Fl::repeat_timeout(0.005, UpdateValues_cb, param);//Ideally 100Hz
        }
    }
    else
    {
        //Get nb of consecutive missed values
        mw->NbMissedUpdates++;
        printf("Nop %d\n", mw->NbMissedUpdates);

        if(mw->Play && (mw->MinWindow->visible() || mw->Window->visible()))
        {
            Fl::repeat_timeout(0.005, UpdateValues_cb, param);
        }
    }
}

//! Check if mouse was recently active (based on MouseActivity: see WinMouseMonitor)
void CheckMouseActivity_cb(void * param)
{
    MainWindow *mw=(MainWindow*)param;

    if(MouseActivity<2)
    {
        printf("Inactive\n");
        mw->MouseActive=false;
    }
    else
    {
        printf("Active\n");
        mw->MouseActive=true;
    }

    //Based on activity:
    //Request pause if needed
    if(!mw->MouseActive && mw->Play)
    {
        PlayPauseButton_cb(NULL, param);
        printf("Inactive: request pause.\n");
    }
    //Request play if needed
    if(mw->MouseActive && !mw->Play)
    {
        PlayPauseButton_cb(NULL, param);
        printf("Active: request play.\n");
    }

    //Reinit
    MouseActivity=0;

    //Check for change every 5s
    Fl::repeat_timeout(5, CheckMouseActivity_cb, param);
}

//!Play/pause: both on device and local (logging and plotting)
void PlayPauseButton_cb(Fl_Widget * widget, void * param)
{
    MainWindow *mw=(MainWindow*)param;

    if(!mw->Play) //Was paused
    {
        //Ask untill success
        while(mw->SerialCom->Connect(true) && !mw->SerialCom->SetState(true))
            Fl::wait(0.1);

        printf("Play/Testing\n");

        //Reset missed values counter
        mw->NbMissedUpdates=0;

        //Start getting values
        Fl::add_timeout(0.1, UpdateValues_cb, param);

        //If no log file openned yet: update log filename and open
        if(!mw->logFile)
        {
            mw->GenerateFilename();
            printf("Log file: %s\n", mw->Filename);
            mw->FilenameInput->value(mw->Filename);
            char fullname[1024+FL_PATH_MAX];
            sprintf(fullname, "%s%s", mw->logPath, mw->Filename);
            mw->logFile=fopen(fullname, "w");
        }

        mw->Play=true;
        mw->OnOffBox->color(FL_GREEN);
        mw->TimeLabel->suspended(0);//Resume timer
        mw->PlayPauseButton->copy_label("@||");
        mw->Window->redraw();
        mw->MinWindow->redraw();
    }
    else //was playing
    {
        //Ask untill sucess
        while(mw->SerialCom->Connect(true) && !mw->SerialCom->SetState(false))
            Fl::wait(0.1);
        printf("Pause\n");

        mw->Play=false;
        mw->OnOffBox->color(FL_YELLOW);
        mw->TimeLabel->suspended(1);//Pause timer
        mw->PlayPauseButton->copy_label("@>");
        mw->Window->redraw();
        mw->MinWindow->redraw();
    }
}

//!Clear button cb: clear graphs
void ClearButton_cb(Fl_Widget * widget, void * param)
{
    MainWindow *mw=(MainWindow*)param;

    //Clear plots and set scale according to mode
    if(mw->Mode=='D')
        mw->Plot->DrawDynamic();
    else
        mw->Plot->DrawStatic();
}

//!Change mode (radio buttons): both local and on device
void ModeGroup_cb(Fl_Widget * widget, void * param)
{
    MainWindow *mw=(MainWindow*)param;

    //Which one is checked
    if(mw->StaticButton->value()==1)
    {
        printf("To static mode... ");
        //Try to set static
        if(mw->SerialCom->SetMode(Static))
        {
            printf("OK.\n");
            //And clear
            ClearButton_cb(widget, param);
        }
        else
        {
            printf("ERROR.\n");
            //Uncheck
            mw->StaticButton->value(0);
        }
    }

    if(mw->DynamicButton->value()==1)
    {
        printf("To dynamic mode... ");
        //Try to set dynamic
        if(mw->SerialCom->SetMode(Dynamic))
        {
            printf("OK.\n");
            //And clear
            ClearButton_cb(widget, param);
        }
        else
        {
            printf("ERROR.\n");
            //Uncheck
            mw->DynamicButton->value(0);
        }
    }
}

//!Close logging and hide window to stop software
void Quit_cb(Fl_Widget * widget, void * param)
{
    MainWindow *mw=(MainWindow*)param;

    //Check if still receiving (meaning that device still ON)
    //and thus remind user to turn it off
    mw->SerialCom->SetState(false);
    fl_message_title("ShoulderTracker");
    fl_alert("Make sure you turned OFF the device !");

    //Close logging if needed
    if(mw->logFile)
        fclose(mw->logFile);

    //Hide window
    mw->MinWindow->hide();
    mw->Play=false;
    mw->SerialCom->Disconnect();
    //everything will be killed
}


//!Try to connect to a device every second quietly
void AutoConnectTimer_cb(void * param)
{
    MainWindow *mw=(MainWindow*)param;

    //Are we really connected ?
    bool receivingValues=true;
    if(mw->NbMissedUpdates>30) //More than 1s without values
    {
        receivingValues=false;
    }

    //Try to connect if not already
    bool connected=mw->SerialCom->Connect(true);

    //If really connected
    if(connected && receivingValues)
    {
        mw->NbMissedConnections=0;

        //If was not running already and not inactive
        if(!mw->Play && mw->MouseActive)
        {
            //Set play, mode and log
            //Set mode
            while(mw->SerialCom->Connect(true) && !mw->SerialCom->SetMode(mw->InitMode))
                Fl::wait(0.1);

            //Start timer
            mw->TimeLabel->suspended(0);

            //Play and open log
            PlayPauseButton_cb(mw->PlayPauseButton, param);
        }
        else
        {
            //Update OnOff label
            mw->OnOffBox->color(FL_GREEN);
            mw->OnOffBox->label("ON");
            mw->OnOffBox->redraw();
        }


        //If first time, show game window and set in test mode
        if(!mw->WasConnected)
        {
            mw->AssessGameWindow->show();
            mw->SerialCom->SetTesting(true);
            mw->WasConnected = true;
        }
    }
    else
    {
        mw->NbMissedConnections++;

        //If we are still connected but not receiving values
        if(connected)
        {
            //then close that connection
            mw->Play=true;
            mw->SerialCom->Disconnect();
            PlayPauseButton_cb(mw->PlayPauseButton, param);
            mw->Play=false;
            mw->NbMissedUpdates=0;

            //and the timer will try to reopen it next round
        }

        //Update OnOff label: rotating symbol
        mw->OnOffBox->color(FL_YELLOW);
        char symb[8][9]={"@8reload", "@9reload", "@6reload", "@3reload", "@2reload", "@1reload", "@4reload", "@7reload"};
        int idx=mw->NbMissedConnections%8;
        mw->OnOffBox->copy_label(symb[idx]);
        mw->OnOffBox->redraw();

        //Suspend timer in both case
        mw->TimeLabel->suspended(1);
    }

    //Repeat
    if(mw->MinWindow->visible())
        Fl::repeat_timeout(.2, AutoConnectTimer_cb, param);
}


//!Prompt to switch between intervention and baseline (therapist use only)
void SetInterventionButton_cb(Fl_Widget * widget, void * param)
{
    MainWindow *mw=(MainWindow*)param;

    fl_message_title("ShoulderTracker");
    if(mw->Intervention)
    {
        if(fl_choice("Are you sure you want to switch to baseline mode?", "Yes, switch", "No, stay in intervention", NULL)==0)
        {
            mw->SetToBaseline();
            fl_alert("Switched to baseline.");
        }
    }
    else
    {
        if(fl_choice("Are you sure you want to switch to intervention mode?", "Yes, switch", "No, stay in baseline", NULL)==0)
        {
            mw->SetToIntervention();
            fl_alert("Switched to intervention.");
        }
    }
}


MainWindow::MainWindow(mode_type init_mode, bool plotting)
{
    InitMode=init_mode;

    //Test version with plotting and controls
        Window=new Fl_Double_Window(800, 400, "Shoulder tracking");
        Window->begin();
            Fl_Tabs *displaytabs=new Fl_Tabs(0, 0, Window->w()-200, Window->h()-20);
            {
                Plot=new Plots(displaytabs->x(), displaytabs->y()+20, displaytabs->w(), displaytabs->h()-20, "Graphs");
            }
            displaytabs->end();
            displaytabs->resizable(Plot);
            Fl_Tabs *tabs=new Fl_Tabs(displaytabs->x()+displaytabs->w(), 0, Window->w()-(displaytabs->x()+displaytabs->w()), Window->h()-20);
            {
                Fl_Group *ControlPanel=new Fl_Group(tabs->x(), tabs->y()+20, tabs->w(), tabs->h()-30, "Control panel");
                    //Play/pause
                    PlayPauseButton = new Fl_Button(ControlPanel->x()+20, ControlPanel->y()+10, 40, 20, "@>");
                    PlayPauseButton->callback(PlayPauseButton_cb, (void*) this);
                    //Clear
                    ClearButton = new Fl_Button(PlayPauseButton->x()+PlayPauseButton->w()+10, PlayPauseButton->y(), 50, 20, "Clear");
                    ClearButton->callback(ClearButton_cb, (void*) this);
                    //Mode
                    ModeGroup = new Fl_Pack(PlayPauseButton->x(), PlayPauseButton->y()+PlayPauseButton->h()+30, 100, 80, "Mode");
                        StaticButton = new Fl_Radio_Round_Button(0, 0, 100, 20, "Static");
                        StaticButton->callback(ModeGroup_cb, (void*) this);
                        DynamicButton = new Fl_Radio_Round_Button(0, 0, 100, 20, "Dynamic");
                        DynamicButton->callback(ModeGroup_cb, (void*) this);
                        if(InitMode==Static)
                        {
                            StaticButton->value(1);
                            DynamicButton->value(0);
                        }
                        else
                        {
                            StaticButton->value(0);
                            DynamicButton->value(1);
                        }
                    ModeGroup->end();
                    ModeGroup->box(FL_EMBOSSED_FRAME);
                    //Log file
                    FilenameInput = new Fl_File_Input(ModeGroup->x()+30, ModeGroup->y()+ModeGroup->h()+10, 120, 30, "Log:");
                    GenerateFilename();
                    printf("File: %s\n", Filename);
                    FilenameInput->value(Filename);
                    logFile=NULL; //Not open yet
                ControlPanel->end();
                ControlPanel->resizable(NULL);
                tabs->resizable(ControlPanel);
            }
            tabs->end();
            StatusBar=new Fl_Box(0, Window->h()-20, Window->w(), 20, "");
            StatusBar->box(FL_EMBOSSED_BOX);

        Window->end();

        //Make window resizable
        Window->resizable(displaytabs);
        Window->size_range(400, 300, 0, 0, 0, 0, 0);


    //The minimalist (patient) version
        //Create an FL_Window at the bottom right of the main screen working area
        int winW=400, winH=310, winX=Fl::x()+Fl::w()-winW, winY=Fl::y()+Fl::h()-winH;
        MinWindow = new Fl_Double_Window(winX, winY, winW, winH, "ShoulderTracker");
        MinWindow->begin();
        //A box for border
        TitleBox = new Fl_Box(0, 0, winW, 40, "ShoulderTracker");
        TitleBox->align(FL_ALIGN_CENTER);
        TitleBox->labelsize(16);
        TitleBox->box(FL_PLASTIC_UP_BOX);
        TitleBox->color(FL_BLUE);

        //Ok indicator
        OnOffBox = new Fl_Box((winW-120)/2., TitleBox->y()+TitleBox->h()+20, 120, 120);
        OnOffBox->align(FL_ALIGN_CENTER);
        OnOffBox->box(FL_OVAL_BOX);
        OnOffBox->color(FL_RED);
        OnOffBox->label("OFF");
        OnOffBox->labelsize(22);

        //Running time
        TimeLabel = new Fl_TimerSimple(FL_VALUE_TIMER, (winW-100)/2., OnOffBox->y()+OnOffBox->h()+10, 100, 40, "");
        TimeLabel->align(FL_ALIGN_CENTER);
        TimeLabel->box(FL_FLAT_BOX);
        TimeLabel->labelsize(18);
        TimeLabel->direction(-1);
        TimeLabel->value(10800000.); //Max time in ms: 3*60*60*1000 => 3h
        TimeLabel->suspended(1);

        //Intervention/baseline
        Preferences = new Fl_Preferences(Fl_Preferences::USER, "ShoulderTrackerIMU", "prefs");
        ReadInterventionState();
        if(Intervention)
            SetInterventionButton = new Fl_Button(winW-80-5, TimeLabel->y()+TimeLabel->h()+5, 80, 40, "Set\nBaseline");
        else
            SetInterventionButton = new Fl_Button(winW-80-5, TimeLabel->y()+TimeLabel->h()+5, 80, 40, "Set\nIntervention");
        SetInterventionButton->callback(SetInterventionButton_cb, (void*)this);

        //Quit button (needed or auto ?)
        QuitButton = new Fl_Button((winW-150)/2., winH-30-5, 150, 30, "QUIT");
        QuitButton->labelsize(16);
        QuitButton->callback(Quit_cb, (void*)this);

        MinWindow->end();
        MinWindow->box(FL_ENGRAVED_BOX);
        MinWindow->border(0);

    NbMissedUpdates=0;
    NbMissedConnections=0;
    MouseActive=true;

    //Either full or minimal window
    if(plotting)
    {
        //Create a serial com with the Arduino
        SerialCom = new Serial();
        Window->show();
    }
    else
    {
        //Create a serial com with the Arduino
        SerialCom = new Serial(true);
        MinWindow->show();
        //Run timer for auto-connect
        Fl::add_timeout(1.0, AutoConnectTimer_cb, (void *)this);
    }
    AssessGameWindow = new GameWindow(this);
    WasConnected = false;
    AssessGameWindow->hide(); //Wait for device to connect to show it

    //Add mouse activity management timer (every 5s)
    Fl::add_timeout(5, CheckMouseActivity_cb, (void *)this);
}

MainWindow::~MainWindow()
{
    delete Preferences;
    //Stop transmission, close connection and destroy Serial
    delete SerialCom;
}


void MainWindow::GenerateFilename()
{
    //Get log path
    Fl_Preferences prefs(Fl_Preferences::USER, "ShoulderTrackerIMU", "logs" );
    prefs.getUserdataPath(logPath, FL_PATH_MAX);
    printf("Logs path: %s\n", logPath);

    //Date and time
    char timestr[80];
    time_t rawtime;
    tm* timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(timestr, 80, "%Y-%m-%d-%H-%M-%S\0", timeinfo);

    //Add prefix and extension
    sprintf(Filename, "STLog_%s.csv", timestr);
}

//! Read preferences to know if in intervention or baseline and set the internal IsIntervention flag (and return true if intervention)
bool MainWindow::ReadInterventionState()
{
    Intervention = false;
    int val;
    Preferences->get("IsIntervention", val, (int)Intervention);
    Intervention = (bool)val;
    return Intervention;
}

void MainWindow::SetToIntervention()
{
    Intervention = true;
    Preferences->set("IsIntervention", (int)Intervention);
    Preferences->flush();
    SetInterventionButton->label("Set\nBaseline");
    SetInterventionButton->redraw();
}

void MainWindow::SetToBaseline()
{
    Intervention = false;
    Preferences->set("IsIntervention", (int)Intervention);
    Preferences->flush();
    SetInterventionButton->label("Set\nIntervention");
    SetInterventionButton->redraw();
}
