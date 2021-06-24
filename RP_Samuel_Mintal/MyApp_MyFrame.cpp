#ifndef WX_PRECOMP
#include <wx/wx.h>
#endif

#include "MyApp_MyFrame.h"



// MyApp is the main Object of this program
wxIMPLEMENT_APP(MyApp);


bool MyApp::OnInit()
{
    MyFrame* frame = new MyFrame();
    frame->Show(true);
    return true;
}

MyFrame::MyFrame()
    : wxFrame(NULL, wxID_ANY, "MAPF Errors Simulator")
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

    simulaton_timer.Bind(wxEVT_TIMER, &MyFrame::thread_simulation_step, this);
}

void MyFrame::OnExit(wxCommandEvent& event) {
    Close(true);
}

void MyFrame::update_panels_data() {
    panel_agents->update_data();
    panel_extended_controls->update_data();
}

void MyFrame::reload_panels_data() {
    panel_agents->update_data();
    panel_extended_controls->reload_data();
    agents_checkboxes_panel->reload_data();

    panel_simulation->paintNow();
}

void MyFrame::thread_simulation_step(wxTimerEvent& event) {

    //I dont care about the event's data
    thread_simulation_step();
}

void MyFrame::thread_simulation_step() {

    //every step move by 70 miliseconds
    current_time_of_simulation += 70; 

    //moved == true if at least one agent moved
    bool moved = simulation.move_to_time(current_time_of_simulation);
    update_panels_data();

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
        

        simulation.load_plans(OpenDialog.GetPath().ToStdString(), actions_duration);


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

void MyFrame::On_button_about(wxCommandEvent& event) {


    auto plan = simulation.get_expected_plan_from_time(140000, simulation.show_agents()[0]);
    simulation.show_agents()[0].set_altered_plan(std::move(plan));
    reload_panels_data();

    //Sort of super quick quide
    wxMessageBox("This is a MAPF error simulator. \n At first you need to load a plan and set action durations. \n Then set desired errors for agents \n And lastly start the simulation",
        "About MAPF error simulator", wxOK | wxICON_INFORMATION);
}