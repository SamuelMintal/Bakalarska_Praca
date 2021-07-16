#pragma once

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#include <wx/wx.h>
#endif

#include <wx/notebook.h>
//#include "RP_icon.XPM" nefunguje

#include "simulation.hpp"
#include "Enums.h"
#include "Custom_Panels_and_Dialogs.h"

//C++ std stuff
#include <iostream>
#include <fstream>



class MyApp : public wxApp
{
public:
    virtual bool OnInit();
};



class MyFrame : public wxFrame {

public:
    MyFrame();

private:

    friend class Draw_Panel;
    friend class Error_Dialog;
    friend class Agent_Info_Panel;
    friend class Extended_controls_panel;
    friend class Buttons_Panel;
    friend class Agents_Plans_Panel;
    friend class Agents_CheckBoxes_Panel;

    void OnStop(wxCommandEvent& event);
    void On_button_load_plan(wxCommandEvent& event);    
    void On_button_set_agents_errors(wxCommandEvent& event);
    void On_button_about(wxCommandEvent& event);
    void OnExit(wxCommandEvent& event);
    void OnGoPause(wxCommandEvent& event);
    void OnChangeMethod(wxCommandEvent& event);
    void OnRestore(wxCommandEvent& event);


    void thread_simulation_step(wxTimerEvent& event); //Being called by wxTimer
    void thread_simulation_step(); //For calling it manualy
    void update_panels_data();
    void reload_panels_data();

    
    // Dialogs and Panels
    Buttons_Panel* panel_buttons = nullptr;
    Agent_Info_Panel* panel_agents = nullptr;
    Draw_Panel* panel_simulation = nullptr;
    Extended_controls_panel* panel_extended_controls = nullptr;
    Error_Dialog* dialog_robot_errors = nullptr;
    Agents_CheckBoxes_Panel* agents_checkboxes_panel = nullptr;
    wxNotebook* notebook = nullptr;

    // Sizers
    wxBoxSizer* sizer_frame_horizontal = nullptr;
    wxBoxSizer* sizer_frame_vertical_final = nullptr;


    //Simulation and variables related to it
    Simulation simulation;
    size_t current_time_of_simulation = 0;
    wxTimer simulaton_timer; //initialized as stopped   

    //Collision detection related stuff
    bool made_backup = false;
    Simulation backup;
}; 
