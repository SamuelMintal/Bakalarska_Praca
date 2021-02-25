#pragma once

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#include <wx/wx.h>
#endif

#include <wx/grid.h>

#include "MyApp_MyFrame.h"
#include "Enums.h"
#include "simulation.h"

class MyFrame;

//Class for setting Errors to one or multiple agents
class Error_Dialog : public wxDialog {

    //All 5 sizers I will need for this Dialog
    wxBoxSizer* input_fields_sizer = nullptr;
    wxBoxSizer* listbox_sizer = nullptr;
    wxBoxSizer* buttons_sizer = nullptr;
    wxBoxSizer* input_fields_and_listbox_sizer = nullptr; //This sizer only helps all_sizer to achieve desired sizing. It does NOT have it's own init functions as other sizers do
    wxBoxSizer* all_sizer = nullptr;


    //Buttons
    wxButton* confirm_button = nullptr;
    wxButton* reject_button = nullptr;


    //speed txtctrls
    wxTextCtrl* txtctrl_speed_max = nullptr;
    wxTextCtrl* txtctrl_speed_sigma = nullptr;
    //angle txtctrls
    wxTextCtrl* txtctrl_angle_max = nullptr;
    wxTextCtrl* txtctrl_angle_sigma = nullptr;
    wxTextCtrl* txtctrl_angle_fatal = nullptr;
    //listbox
    wxListBox* list_box = nullptr;


    //speed txts
    wxStaticText* txt_speed_max = nullptr;
    wxStaticText* txt_speed_sigma = nullptr;
    //angle txts
    wxStaticText* txt_angle_max = nullptr;
    wxStaticText* txt_angle_sigma = nullptr;
    wxStaticText* txt_angle_fatal = nullptr;
    //Choosing txt
    wxStaticText* txt_choose_agents_names = nullptr;


    //Where to look for simulation
    MyFrame* Frame_with_simulation = nullptr;

    void init_input_fields_sizer();
    void init_buttons_sizer();
    void init_listbox_sizer();
    void init_all_sizer();

    void potentialy_display_properties_of_agent(wxCommandEvent& event);
    int safe_stoi(const wxTextCtrl* curr_txtctrl, int default_val);

    void Confirmed(wxCommandEvent& event);
    void Rejected(wxCommandEvent& event);

public:

    Error_Dialog(wxWindow* parent, MyFrame* Frame_with_simulation);
};


class Agent_Info_Panel : public wxPanel {
    MyFrame* Frame_with_simulation = nullptr;
    wxBoxSizer* agent_info_sizer = nullptr;
    wxGrid* agent_grid;

public:

    Agent_Info_Panel(wxWindow* parent, MyFrame* Frame_with_simulation, int id, const wxPoint& position, const wxSize& size);


    /* Updates it's data from simulation
    * Call when simulation state changed.
    * E.g. simulation.move_to_time() or simulation.load_plan()
    */
    void update_data();
};


class Agents_Plans_Panel : public wxScrolledWindow
{
    //Sizer
    wxSizer* Agents_Plans_Panel_sizer = nullptr;

    //Grid of Plans of Agents
    //Vector's destructor also destroys wxGrids
    //Thus is important that those wxGrids have NULL as theyr parent. Else both vector and wxWidgets would try to destroy it and that would be bad...
    std::vector<wxGrid*> plans_of_agents;

    //Frame with simulation...
    MyFrame* Frame_with_simulation = nullptr;

    //Collumn width sizer
    float scaler = 0.5;

    //Adds plans of agent into the plans_of_agents
    void add_plans_of_agents(const Agent& agent);

    wxGrid* create_grid_from_plan(const std::vector<plan_step>& plan, std::string label);

public:

    Agents_Plans_Panel(wxWindow* parent, MyFrame* Frame_with_simulation);

    void update_data();

};


class Extended_controls_panel : public wxPanel {
    MyFrame* Frame_with_simulation = nullptr;
    wxBoxSizer* extended_controls_panel_sizer = nullptr;
    wxSlider* slider_current_time_of_simulation = nullptr;
    Agents_Plans_Panel* agents_plans_panel = nullptr;

    /*
    * Sets the Frame_with_simulation->current_time_of_simulation to the current slider value and refreshes the simulation panel so it shows
    */
    void On_slider_scroll(wxEvent& event);

public:

    Extended_controls_panel(wxWindow* parent, MyFrame* Frame_with_simulation, int id, const wxPoint& position, const wxSize& size);

    /*
    *  Sets slider to current_time_of_simulation and updates it's maximum value.
    */
    void update_data();
};




class Buttons_Panel : public wxPanel {
    
    wxButton* button_GoPause = nullptr;
    wxButton* button_stop = nullptr;
    wxButton* button_load_plan = nullptr;
    wxButton* button_about = nullptr;
    wxButton* button_set_common_robot_errors = nullptr;
    wxButton* button_set_agents_errors = nullptr;

    wxBoxSizer* sizer_panel_buttons = nullptr;

public:

    Buttons_Panel(wxWindow* parent, MyFrame* Frame_with_simulation, int id, const wxPoint& position, const wxSize& size);

    void set_GoPause_label_to(const std::string& new_label);

};




class Action_durations_Dialog : public wxDialog {

    #define DEFAULT_DURATION 2000

    //Static texts
    wxStaticText* txt_dialogs_title = nullptr;
    wxStaticText* txt_go_duration = nullptr;
    wxStaticText* txt_turn_duration = nullptr;
    wxStaticText* txt_wait_duration = nullptr;

    //Text controls
    wxTextCtrl* txtctrl_go_duration = nullptr;
    wxTextCtrl* txtctrl_turn_duration = nullptr;
    wxTextCtrl* txtctrl_wait_duration = nullptr;    

    //Button
    wxButton* button_set = nullptr;

    //Sizer
    wxSizer* Action_durations_sizer = nullptr;

    void init_Action_durations_sizer(std::vector<int>);

public:
    
   
    Action_durations_Dialog();

    /*
    * default_values[0] == go_duration,  default_values[1] == turn_duration, default_values[2] == wait_duration
    */
    Action_durations_Dialog(wxWindow* parent, int id, std::vector<int> default_values, const wxSize& size);

    /*
    * returned[0] == go_duration, returned[1] == turn_duration, returned[2] == wait_duration
    */
    std::vector<int> get_data();
    int safe_stoi(const wxTextCtrl* curr_txtctrl, int default_val);
};



class Draw_Panel : public wxPanel
{
    MyFrame* MyFrame_with_simulation;

    const int agent_radius = 20;            // Parameters for map drawing 
    int scale = 50;                         // 
    const int offset = scale + agent_radius;//

    wxPen pen_map_lines = wxPen(wxColor(0, 0, 0), 5);
    wxPen pen_map_obstacles = wxPen(wxColour(255, 255, 255), 5);
    wxPen pen_agents = wxPen(wxColour(255, 255, 255), 5);
    wxPen pen_error_ring_0 = wxPen(wxColour(0, 255, 0), 5);
    wxPen pen_error_ring_1 = wxPen(wxColour(255, 0, 0), 5);    
    wxPen pen_error_ring_5 = wxPen(wxColour(0, 0, 0), 5);
    wxPen pen_rotation = wxPen(wxColour(220, 220, 220), 7);

    int hexChar_to_num(char c);
    wxPoint angle_to_point(int length, int angle, wxPoint base);

public:
    Draw_Panel(MyFrame* parent_simulation, wxFrame* parent, wxWindowID id, wxSize min_size);

    wxColor hex_to_wxColor(std::string hex);

    void paintEvent(wxPaintEvent& evt);
    void paintNow();
    void render(wxDC& dc);    
};



