#ifndef WX_PRECOMP
#include <wx/wx.h>
#endif

#include "MyApp_MyFrame.h"
#include "Testing_code.hpp"


// MyApp is the main Object of this program
wxIMPLEMENT_APP(MyApp);

//#define mintalTESTING

bool MyApp::OnInit()
{
    
#ifdef mintalTESTING
    TEST___hypothesis_1(); //change for cycle range and also hyp 5
    TEST___hypothesis_4();
#endif 

    MyFrame* frame = new MyFrame();
    frame->Show(true);

    return true;
}

/*Initalization of the whole application
*/
MyFrame::MyFrame()
    : wxFrame(NULL, wxID_ANY, "MAPF Errors and Collision Detection Simulator")
{
    //Setting up notebook
    notebook = new wxNotebook(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxNB_TOP);

    //setting up panels in notebook and adding them there
    panel_agents = new Agent_Info_Panel(notebook, this, ID_panel_agents, wxDefaultPosition, wxSize(100, 100));
    agents_checkboxes_panel = new Agents_CheckBoxes_Panel(notebook, this);
    notebook->AddPage(panel_agents, "Agents Info", true);
    notebook->AddPage(agents_checkboxes_panel, "Expected Course");
    

    //Setting up panels in frame    
    panel_buttons = new Buttons_Panel(this, this, ID_panel_buttons, wxDefaultPosition, wxSize(100, 100));
    panel_simulation = new Draw_Panel(this, this, ID_panel_simulation, wxSize(500, 200));    
    panel_extended_controls = new Extended_controls_panel(this, this, ID_panel_extended_controls, wxDefaultPosition, wxSize(20, 20));

    //Adding them into the first (horizontal) sizer
    sizer_frame_horizontal = new wxBoxSizer(wxHORIZONTAL);
    sizer_frame_horizontal->Add(panel_buttons, 0, wxLEFT | wxEXPAND | wxALL, 10);
    sizer_frame_horizontal->Add(panel_simulation, 3, wxRIGHT | wxEXPAND | wxALL, 10);         
    sizer_frame_horizontal->Add(notebook, 0, wxRIGHT | wxEXPAND | wxALL, 10);
    

    //Group them into the main (vertical) sizer
    sizer_frame_vertical_final = new wxBoxSizer(wxVERTICAL);
    sizer_frame_vertical_final->Add(sizer_frame_horizontal, 5, wxEXPAND | wxALL, 10);
    sizer_frame_vertical_final->Add(panel_extended_controls, 2, wxEXPAND | wxRIGHT | wxLEFT | wxDOWN, 20);

    //at last set frame sizer to fit
    SetSizerAndFit(sizer_frame_vertical_final);

    //Bindings for the buttons. Events with these IDs are emitted from buttons from Buttons_Panel
    Bind(wxEVT_BUTTON, &MyFrame::On_button_about, this, ID_button_about);
    Bind(wxEVT_BUTTON, &MyFrame::OnGoPause, this, ID_button_GoPause);
    Bind(wxEVT_BUTTON, &MyFrame::OnStop, this, ID_button_Stop);        
    Bind(wxEVT_BUTTON, &MyFrame::On_button_set_agents_errors, this, ID_button_set_agents_errors);
    Bind(wxEVT_BUTTON, &MyFrame::On_button_load_plan, this, ID_button_load_plan);
    Bind(wxEVT_BUTTON, &MyFrame::OnChangeMethod, this, ID_button_change_detection_method);
    Bind(wxEVT_BUTTON, &MyFrame::OnRestore, this, ID_button_restore_backup);

    simulaton_timer.Bind(wxEVT_TIMER, &MyFrame::thread_simulation_step, this);
}

void MyFrame::OnExit(wxCommandEvent& event) {
    Close(true);
}

/* when simulation state changes this function should be called
* Othervise the panels will not be consistent with the simulation
*/
void MyFrame::update_panels_data() {
    panel_agents->update_data();
    panel_extended_controls->update_data();
}

/* when we set new simulation this function should be called
* Othervise the panels will not be consistent with the simulation
*/
void MyFrame::reload_panels_data() {
    panel_agents->update_data();
    panel_extended_controls->reload_data();
    agents_checkboxes_panel->reload_data();

    panel_simulation->paintNow();
}

/* moves simulation to next time
*/
void MyFrame::thread_simulation_step(wxTimerEvent& event) {

    //I dont care about the event's data
    thread_simulation_step();
}

void MyFrame::thread_simulation_step() {

    //every step move by 70 miliseconds
    current_time_of_simulation += 70; 

    //moved == true if at least one agent moved
    bool moved = simulation.move_to_time(current_time_of_simulation);

    if (simulation.last_detection_result.collision_detected && !made_backup) {

        simulaton_timer.Stop();

        std::string agt1_name = simulation.show_agents()[simulation.last_detection_result.agent1_index].get_name();
        std::string agt2_name = simulation.show_agents()[simulation.last_detection_result.agent2_index].get_name();

        std::string from_time = std::to_string(simulation.last_detection_result.from_time);
        std::string to_time = std::to_string(simulation.last_detection_result.to_time);

        std::string collision_info_text = "Collision will occur between agents " + agt1_name + " and " + agt2_name + " in time range from " + from_time + " to " + to_time + ".\n";

        //Notify user that collision will be shown
        wxMessageBox(collision_info_text + "Agents will switch to expected plans and collision will be shown.\nTo rollback to current simulation state you can press Restore Backup button.",
            "Collision detected", wxOK | wxICON_INFORMATION);
        simulaton_timer.Start();

        made_backup = true;
        panel_buttons->set_enable_disable_backup_button(true);
        backup = simulation;

        simulation.set_to_expected_plans_state(current_time_of_simulation);
        reload_panels_data();
    }
    else {
        update_panels_data();
    }
        
    panel_simulation->paintNow();

    if(!moved) {        
        simulaton_timer.Stop();
        panel_buttons->set_GoPause_label_to("Go");
    }

}

void MyFrame::OnGoPause(wxCommandEvent& event) {

    if (simulaton_timer.IsRunning()) {
        simulaton_timer.Stop();
        panel_buttons->set_GoPause_label_to("Go");
    }
    else {
        simulaton_timer.Start(80); //step every 80 miliseconds
        panel_buttons->set_GoPause_label_to("Pause");
    }
}


void MyFrame::OnStop(wxCommandEvent& event) {

    simulaton_timer.Stop();

    current_time_of_simulation = 0;
    simulation.move_to_time(0);
    update_panels_data();

    panel_simulation->paintNow();

    panel_buttons->set_GoPause_label_to("Go");
}

void  MyFrame::On_button_load_plan(wxCommandEvent& event) {
    wxFileDialog OpenDialog(nullptr, "Choose a .solr file to load", wxEmptyString, wxEmptyString, _("Plan files (*.solr)|*.solr"), wxFD_OPEN, wxDefaultPosition);

    if (OpenDialog.ShowModal() == wxID_OK) { // if the user clicked "Open" instead of "cancel"

        Action_durations_Dialog dur_dialog;
        dur_dialog.ShowModal(); //Retrievs the data even if user clicked X
        auto actions_duration = dur_dialog.get_data();
        
        Collision_detection_Dialog coll_diag;
        coll_diag.ShowModal();        

        simulation.load_plans(OpenDialog.GetPath().ToStdString(), actions_duration);
        simulation.chosen_detection_method = coll_diag.get_data();

        if (simulaton_timer.IsRunning()) {
            simulaton_timer.Stop();
            //button_GoPause->SetLabel("Go");
            panel_buttons->set_GoPause_label_to("Go");
        }
        current_time_of_simulation = 0;
        reload_panels_data();
    }
    panel_simulation->Refresh();
}

void MyFrame::On_button_set_agents_errors(wxCommandEvent& event) {

    dialog_robot_errors = new Error_Dialog(this, this);
    dialog_robot_errors->ShowModal();
    reload_panels_data();
}

void MyFrame::OnChangeMethod(wxCommandEvent& event) {
    Collision_detection_Dialog coll_diag;
    coll_diag.ShowModal();
    simulation.chosen_detection_method = coll_diag.get_data();
}

void MyFrame::OnRestore(wxCommandEvent& event) {
    
    simulation = backup;

    current_time_of_simulation = 0;
    simulation.move_to_time(0);

    reload_panels_data();

    made_backup = false;
    panel_buttons->set_enable_disable_backup_button(false);    
}

void MyFrame::On_button_about(wxCommandEvent& event) {

    //Sort of super quick quide
    wxMessageBox("This is a MAPF error simulator. \n At first you need to load a plan and set action durations. \n Then set desired errors for agents \n And lastly start the simulation",
        "About MAPF error simulator", wxOK | wxICON_INFORMATION);
}