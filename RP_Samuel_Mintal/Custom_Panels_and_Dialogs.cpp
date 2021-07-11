#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#include <wx/wx.h>
#endif

#include "Custom_Panels_and_Dialogs.h"


/*
* 
**** class Error_Dialog : public wxDialog
* 
*/
 
    void Error_Dialog::init_input_fields_sizer() {

        //Make input fields sizer and put all of input fields into him
        input_fields_sizer = new wxBoxSizer(wxVERTICAL);

        txt_speed_max = new wxStaticText(this, wxID_ANY, "Maximum speed error [ms]:");
        txtctrl_speed_max = new wxTextCtrl(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0L, wxTextValidator(wxFILTER_NUMERIC)); // only accepts numbers       
        input_fields_sizer->Add(txt_speed_max, 1, wxEXPAND);
        input_fields_sizer->Add(txtctrl_speed_max, 1, wxEXPAND);

        txt_speed_sigma = new wxStaticText(this, wxID_ANY, "Sigma for normal distribution of speed error:", wxDefaultPosition, wxDefaultSize);
        txtctrl_speed_sigma = new wxTextCtrl(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0L, wxTextValidator(wxFILTER_NUMERIC)); // only accepts numbers
        input_fields_sizer->Add(txt_speed_sigma, 1, wxEXPAND);
        input_fields_sizer->Add(txtctrl_speed_sigma, 1, wxEXPAND);

        txt_angle_max = new wxStaticText(this, wxID_ANY, "Maximum angle error [degrees]:");
        txtctrl_angle_max = new wxTextCtrl(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0L, wxTextValidator(wxFILTER_NUMERIC)); // only accepts numbers
        input_fields_sizer->Add(txt_angle_max, 1, wxEXPAND);
        input_fields_sizer->Add(txtctrl_angle_max, 1, wxEXPAND);

        txt_angle_sigma = new wxStaticText(this, wxID_ANY, "Sigma for normal distribution of angle error:");
        txtctrl_angle_sigma = new wxTextCtrl(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0L, wxTextValidator(wxFILTER_NUMERIC)); // only accepts numbers
        input_fields_sizer->Add(txt_angle_sigma, 1, wxEXPAND);
        input_fields_sizer->Add(txtctrl_angle_sigma, 1, wxEXPAND);

        txt_angle_fatal = new wxStaticText(this, wxID_ANY, "Fatal angle [degrees]:");
        txtctrl_angle_fatal = new wxTextCtrl(this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0L, wxTextValidator(wxFILTER_NUMERIC)); // only accepts numbers
        input_fields_sizer->Add(txt_angle_fatal, 1, wxEXPAND);
        input_fields_sizer->Add(txtctrl_angle_fatal, 1, wxEXPAND);
    }

    void Error_Dialog::init_buttons_sizer() {
        buttons_sizer = new wxBoxSizer(wxHORIZONTAL);
        confirm_button = new wxButton(this, ID_button_set_agents_errors_CONFIRM, "Confirm", wxDefaultPosition, wxSize(150, 50));
        reject_button = new wxButton(this, ID_button_set_agents_errors_REJECT, "Reject", wxDefaultPosition, wxSize(150, 50));


        buttons_sizer->AddSpacer(25);
        buttons_sizer->Add(confirm_button, 1, wxEXPAND);
        buttons_sizer->AddSpacer(25);
        buttons_sizer->Add(reject_button, 1, wxEXPAND);
        buttons_sizer->AddSpacer(25);
    }

    void Error_Dialog::init_listbox_sizer() {
        if (!Frame_with_simulation)
            return;

        wxArrayString wx_agents_names;
        std::vector<std::string> vct_agent_names = Frame_with_simulation->simulation.get_agents_names();

        //Get data from vector to WX's wxArrayString which it uses for listboxes
        for (size_t i = 0; i < vct_agent_names.size(); i++)
            wx_agents_names.Add(vct_agent_names[i]);

        //Initialize items which are meant to be in listbox_sizer
        list_box = new wxListBox(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wx_agents_names, wxLB_MULTIPLE);
        txt_choose_agents_names = new wxStaticText(this, wxID_ANY, "Choose Names of agents whose characteristics are to be changed");

        listbox_sizer = new wxBoxSizer(wxVERTICAL);

        listbox_sizer->Add(txt_choose_agents_names, 1, wxRIGHT, 20);
        listbox_sizer->Add(list_box, 1, wxEXPAND | wxRIGHT, 20);
    }

    void Error_Dialog::init_all_sizer() {

        //Use the helping sizer to set input_fields and listbox Horizontaly to each other
        input_fields_and_listbox_sizer = new wxBoxSizer(wxHORIZONTAL);

        input_fields_and_listbox_sizer->AddSpacer(25);
        input_fields_and_listbox_sizer->Add(listbox_sizer, 1, wxLEFT);
        input_fields_and_listbox_sizer->AddSpacer(100);
        input_fields_and_listbox_sizer->Add(input_fields_sizer, 1, wxRIGHT);
        input_fields_and_listbox_sizer->AddSpacer(25);

        //And finaly put all sizers into the main one
        all_sizer = new wxBoxSizer(wxVERTICAL);
        all_sizer->AddSpacer(15);
        all_sizer->Add(input_fields_and_listbox_sizer, 10, wxTOP);
        all_sizer->Add(buttons_sizer, 1, wxDOWN);
        all_sizer->AddSpacer(15);
    }

    void Error_Dialog::potentialy_display_properties_of_agent(wxCommandEvent& event) {

        wxArrayInt listbox_wx_indexes;
        int count = list_box->GetSelections(listbox_wx_indexes);

        //If only one agent is selected display his parameters into text_controls
        if (count == 1) {

            std::string desired_agent_name = list_box->GetString(listbox_wx_indexes[0]).ToStdString();
            auto const_agents = Frame_with_simulation->simulation.show_agents();

            for (auto& agent : const_agents) {

                //If I found the agent which was clicked by user
                if (agent.get_name() == desired_agent_name) {
                    auto err_vec = agent.get_err_vector();

                    txtctrl_speed_max->SetValue(std::to_string(err_vec[0]));
                    txtctrl_speed_sigma->SetValue(std::to_string(err_vec[1]));
                    txtctrl_angle_max->SetValue(std::to_string(err_vec[2]));
                    txtctrl_angle_sigma->SetValue(std::to_string(err_vec[3]));
                    txtctrl_angle_fatal->SetValue(std::to_string(err_vec[4]));

                    //It is all I wanted to do so break, which will result in returning from function
                    break;
                }
            }

        }
        else {
            //Set all text_controls to empty string
            txtctrl_speed_max->SetValue("");
            txtctrl_speed_max->SetValue("");
            txtctrl_speed_sigma->SetValue("");
            txtctrl_angle_max->SetValue("");
            txtctrl_angle_sigma->SetValue("");
            txtctrl_angle_fatal->SetValue("");
        }
    }

    int Error_Dialog::safe_stoi(const wxTextCtrl* curr_txtctrl, int default_val = 0) {
        if (curr_txtctrl->GetValue().ToStdString() == "")
            return default_val;

        return stoi(curr_txtctrl->GetValue().ToStdString());
    }

    void Error_Dialog::Confirmed(wxCommandEvent& event) {

        std::vector<int> ret; // 0 = speed max, 1 = speed sigma, 2 = angle max, 3 = angle sigma, fatal angle
        ret.push_back(safe_stoi(txtctrl_speed_max));
        ret.push_back(safe_stoi(txtctrl_speed_sigma));
        ret.push_back(safe_stoi(txtctrl_angle_max));
        ret.push_back(safe_stoi(txtctrl_angle_sigma));
        ret.push_back(safe_stoi(txtctrl_angle_fatal));


        //Get all indexes which are selected into listbox_indexes vector
        wxArrayInt listbox_wx_indexes;
        std::vector<int> listbox_indexes;
        int count = list_box->GetSelections(listbox_wx_indexes);

        for (int i = 0; i < count; i++)
            listbox_indexes.push_back(listbox_wx_indexes[i]);


        //Set errors of the selected agents
        for (size_t i = 0; i < listbox_indexes.size(); i++) {

            std::string curr_agent_name = list_box->GetString(listbox_indexes[i]).ToStdString();
            Frame_with_simulation->simulation.set_errors_to_agent(curr_agent_name, ret);
        }

        this->Close();
    }


    void Error_Dialog::Rejected(wxCommandEvent& event) {

        int answer = wxMessageBox("Are you sure you want to leave without saveing changes?", "Unsaved data:",
            wxYES | wxNO);

        if (answer == wxYES)
            this->Close();

        //Else dont leave
    }



    Error_Dialog::Error_Dialog(wxWindow* parent, MyFrame* Frame_with_simulation) :
        wxDialog(parent, wxID_ANY, "Choose errors for agents"),
        Frame_with_simulation(Frame_with_simulation) {

        init_input_fields_sizer();
        init_buttons_sizer();
        init_listbox_sizer();
        init_all_sizer();

        SetSizerAndFit(all_sizer);

        Bind(wxEVT_BUTTON, &Error_Dialog::Confirmed, this, ID_button_set_agents_errors_CONFIRM);
        Bind(wxEVT_BUTTON, &Error_Dialog::Rejected, this, ID_button_set_agents_errors_REJECT);

        //Bind showing agents properties, if he is the only one selected
        list_box->Bind(wxEVT_LISTBOX, &Error_Dialog::potentialy_display_properties_of_agent, this);
    }







/*
*
**** class Agent_Info_Panel : public wxPanel
*
*/  

    Agent_Info_Panel::Agent_Info_Panel(wxWindow* parent, MyFrame* Frame_with_simulation, int id, const wxPoint& position, const wxSize& size) :
        wxPanel(parent, id, position, size),
        Frame_with_simulation(Frame_with_simulation) {

        agent_grid = new wxGrid(this, wxID_ANY);
        agent_grid->CreateGrid(0, 3);
        agent_grid->HideRowLabels();
        agent_grid->EnableEditing(false);

        agent_grid->SetColLabelValue(0, "Agent name");
        agent_grid->SetColLabelValue(1, "Error");
        agent_grid->SetColLabelValue(2, "Time diff");

        agent_info_sizer = new wxBoxSizer(wxVERTICAL);
        agent_info_sizer->Add(agent_grid, wxEXPAND | wxALL);
        SetSizerAndFit(agent_info_sizer);

    }


    /* Updates it's data from simulation
    * Call when simulation state changed.
    * E.g. simulation.move_to_time() or simulation.load_plan()
    */
    void Agent_Info_Panel::update_data() {

        //Delete all the current data so I can append new
        if (agent_grid->GetNumberRows())
            agent_grid->DeleteRows(0, agent_grid->GetNumberRows());

        auto const_agents = Frame_with_simulation->simulation.show_agents();
        agent_grid->AppendRows(const_agents.size());
        
        for (size_t i = 0; i < const_agents.size(); i++) {

            agent_grid->SetCellValue(i, 0, const_agents[i].get_name());
            agent_grid->SetCellBackgroundColour(i, 0, Draw_Panel::hex_to_wxColor(const_agents[i].get_color()));

            if (const_agents[i].get_error_state() == 1) {
                agent_grid->SetCellValue(i, 1, "Error");
                agent_grid->SetCellBackgroundColour(i, 1, wxColor("red"));
                agent_grid->SetCellBackgroundColour(i, 2, wxColor("red"));
            }
            else if (const_agents[i].get_error_state() == 0) {
                agent_grid->SetCellValue(i, 1, "No Error");
                agent_grid->SetCellBackgroundColour(i, 1, wxColor("green"));
                agent_grid->SetCellBackgroundColour(i, 2, wxColor("green"));
            }
            else if (const_agents[i].get_error_state() == 5) {
                agent_grid->SetCellValue(i, 1, "Lost");
                agent_grid->SetCellBackgroundColour(i, 1, wxColor("black"));
                agent_grid->SetCellBackgroundColour(i, 2, wxColor("black"));
            }

            agent_grid->SetCellValue(i, 2, std::to_string(Frame_with_simulation->simulation.get_time_diffs_of_agent_at_time(i, Frame_with_simulation->current_time_of_simulation)));
        }
    }






/*
*
**** class Extended_controls_panel : public wxPanel
*
*/

    /*
    * Sets the Frame_with_simulation->current_time_of_simulation to the current slider value and refreshes the simulation panel so it shows
    */
    void Extended_controls_panel::On_slider_scroll(wxEvent& event) {
        
        Frame_with_simulation->current_time_of_simulation = slider_current_time_of_simulation->GetValue();
        Frame_with_simulation->thread_simulation_step();
    }

        

    Extended_controls_panel::Extended_controls_panel(wxWindow* parent, MyFrame* Frame_with_simulation, int id, const wxPoint& position, const wxSize& size) :
        wxPanel(parent, id, position, size),
        Frame_with_simulation(Frame_with_simulation) {
        extended_controls_panel_sizer = new wxBoxSizer(wxVERTICAL);
               
        slider_current_time_of_simulation = new wxSlider(this, wxID_ANY, 0, 0, Frame_with_simulation->simulation.get_agent_plan_max_length() + 1, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL | wxSL_LABELS); //+1 because empty plan could have 0 max and slider requires that min < max        
        extended_controls_panel_sizer->Add(slider_current_time_of_simulation, 0, wxEXPAND | wxUP | wxLEFT | wxRIGHT | wxDOWN, 10);
        

        agents_plans_panel = new Agents_Plans_Panel(this, Frame_with_simulation);
        extended_controls_panel_sizer->Add(agents_plans_panel, 1, wxEXPAND);


        SetSizerAndFit(extended_controls_panel_sizer);
        slider_current_time_of_simulation->Bind(wxEVT_SLIDER, &Extended_controls_panel::On_slider_scroll, this);
    }


    /* Sets slider to current_time_of_simulation.
    *  Also forces Agents_Plans_Panel to reload it's grids.
    * Call when you reload the simulation
    */
    void Extended_controls_panel::update_data() {

        slider_current_time_of_simulation->SetValue(Frame_with_simulation->current_time_of_simulation);
        agents_plans_panel->update_simulation_current_time_line();

    }

    /* Sets slider to current_time_of_simulation and updates it's maximum value.
    * Moves simulation acutal time line of Agents_Plans_Panel
    * Call when simulation moves, but doesnt reload.
    */
    void Extended_controls_panel::reload_data() {

        //+1 because empty plan could have 0 max and slider requires that min < max
        slider_current_time_of_simulation->SetMax(Frame_with_simulation->simulation.get_agent_plan_max_length() + 1); 
        //slider_current_time_of_simulation->SetValue(Frame_with_simulation->current_time_of_simulation);

        agents_plans_panel->update_data();
    }







/*
*
**** class Buttons_Panel : public wxPanel
*
*/
    Buttons_Panel::Buttons_Panel(wxWindow* parent, MyFrame* Frame_with_simulation, int id, const wxPoint& position, const wxSize& size) 
        : wxPanel(parent, id, position, size) {

        button_about = new wxButton(this, ID_button_about, "About", wxDefaultPosition, wxSize(150, 50));
        button_GoPause = new wxButton(this, ID_button_GoPause, "Go", wxDefaultPosition, wxSize(150, 50));
        button_stop = new wxButton(this, ID_button_Stop, "Stop", wxDefaultPosition, wxSize(150, 50));        
        button_load_plan = new wxButton(this, ID_button_load_plan, "Load Plan", wxDefaultPosition, wxSize(150, 50));
        button_set_agents_errors = new wxButton(this, ID_button_set_agents_errors, "Set Agents Errors", wxDefaultPosition, wxSize(150, 50));
        button_change_detection_method = new wxButton(this, ID_button_change_detection_method, "Set Detection Method", wxDefaultPosition, wxSize(150, 50));
        button_restore_backup = new wxButton(this, ID_button_restore_backup, "Restore Backup", wxDefaultPosition, wxSize(150, 25));

        sizer_panel_buttons = new wxBoxSizer(wxVERTICAL);
        sizer_panel_buttons->Add(button_about, 1, wxEXPAND | wxALL, 10);
        sizer_panel_buttons->Add(button_GoPause, 1, wxEXPAND | wxALL, 10);
        sizer_panel_buttons->Add(button_stop, 1, wxEXPAND | wxALL, 10);        
        sizer_panel_buttons->Add(button_load_plan, 1, wxEXPAND | wxALL, 10);
        sizer_panel_buttons->Add(button_set_agents_errors, 1, wxEXPAND | wxALL, 10);
        sizer_panel_buttons->Add(button_change_detection_method, 1, wxEXPAND | wxALL, 10);
        sizer_panel_buttons->Add(button_restore_backup, 1, wxEXPAND | wxALL, 10);

        button_restore_backup->Enable(false);

        SetSizerAndFit(sizer_panel_buttons);
    }

    void Buttons_Panel::set_GoPause_label_to(const std::string& new_label) {

        button_GoPause->SetLabel(new_label);
    }

    void Buttons_Panel::set_enable_disable_backup_button(bool value) {

        button_restore_backup->Enable(value);
    }






/*
*
**** class Action_durations_Dialog : public wxDialog
*
*/

    /* Parameterless constructor
    * Use ONLY when the dialog is being constructed on the stack
    */
    Action_durations_Dialog::Action_durations_Dialog() 
        : wxDialog(nullptr, wxID_ANY, "Choose agent's actions durations") {

        init_Action_durations_sizer({ DEFAULT_DURATION, DEFAULT_DURATION, DEFAULT_DURATION });
    }

    /*
    * Use when the dialog is being constructed on the heap, also allows more control
    */
    Action_durations_Dialog::Action_durations_Dialog(wxWindow* parent, int id, std::vector<int> default_values, const wxSize& size)
    : wxDialog(parent, id, "Choose agent's actions durations", wxDefaultPosition, size) {

        //check for correct size of default_values, if it's bad, than change to DEFAULT_DURATION (currently 2000)
        if (default_values.size() != 3)
            default_values = { DEFAULT_DURATION, DEFAULT_DURATION, DEFAULT_DURATION };

        init_Action_durations_sizer(default_values);
    }

    void Action_durations_Dialog::init_Action_durations_sizer(std::vector<int> default_values) {

        //Make input fields sizer and put all of input fields into him
        Action_durations_sizer = new wxBoxSizer(wxVERTICAL);

        Action_durations_sizer->AddSpacer(10);
        txt_dialogs_title = new wxStaticText(this, wxID_ANY, "Now Choose how long every agent's\n  individual actions will take in ms.");
        Action_durations_sizer->Add(txt_dialogs_title, 0, wxALL, 10);

        txt_go_duration = new wxStaticText(this, wxID_ANY, "Go duration:");
        txtctrl_go_duration = new wxTextCtrl(this, wxID_ANY, std::to_string(default_values[0]), wxDefaultPosition, wxDefaultSize, 0L, wxTextValidator(wxFILTER_NUMERIC)); // only accepts numbers       
        Action_durations_sizer->AddSpacer(10);
        Action_durations_sizer->Add(txt_go_duration, 0, wxTOP | wxLEFT | wxRIGHT, 10);
        Action_durations_sizer->Add(txtctrl_go_duration, 1, wxALL | wxEXPAND, 10);

        txt_turn_duration = new wxStaticText(this, wxID_ANY, "Turn duration:");
        txtctrl_turn_duration = new wxTextCtrl(this, wxID_ANY, std::to_string(default_values[1]), wxDefaultPosition, wxDefaultSize, 0L, wxTextValidator(wxFILTER_NUMERIC)); // only accepts numbers       
        Action_durations_sizer->AddSpacer(20);
        Action_durations_sizer->Add(txt_turn_duration, 0, wxTOP | wxLEFT | wxRIGHT, 10);
        Action_durations_sizer->Add(txtctrl_turn_duration, 1, wxALL | wxEXPAND, 10);

        txt_wait_duration = new wxStaticText(this, wxID_ANY, "Wait duration:");
        txtctrl_wait_duration = new wxTextCtrl(this, wxID_ANY, std::to_string(default_values[2]), wxDefaultPosition, wxDefaultSize, 0L, wxTextValidator(wxFILTER_NUMERIC)); // only accepts numbers       
        Action_durations_sizer->AddSpacer(20);
        Action_durations_sizer->Add(txt_wait_duration, 0, wxTOP | wxLEFT | wxRIGHT, 10);
        Action_durations_sizer->Add(txtctrl_wait_duration, 1, wxALL | wxEXPAND, 10);

        button_set = new wxButton(this, wxID_OK, "Set", wxDefaultPosition, wxSize(150,50));
        Action_durations_sizer->Add(button_set, 2, wxEXPAND | wxALL, 10);

        SetSizerAndFit(Action_durations_sizer);
    }

    int Action_durations_Dialog::safe_stoi(const wxTextCtrl* curr_txtctrl, int default_val = 0) {
        if (curr_txtctrl->GetValue().ToStdString() == "")
            return default_val;

        return stoi(curr_txtctrl->GetValue().ToStdString());
    }

    std::vector<int> Action_durations_Dialog::get_data() {
        std::vector<int> ret(3);
        ret[0] = safe_stoi(txtctrl_go_duration, DEFAULT_DURATION);
        ret[1] = safe_stoi(txtctrl_turn_duration, DEFAULT_DURATION);
        ret[2] = safe_stoi(txtctrl_wait_duration, DEFAULT_DURATION);

        return ret;
    }






/*
*
**** class Draw_Panel : public wxPanel
*
*/

    Draw_Panel::Draw_Panel(MyFrame* parent_simulation, wxFrame* parent, wxWindowID id, wxSize min_size) 
        : wxPanel(parent, id, wxDefaultPosition, min_size, wxFULL_REPAINT_ON_RESIZE) {
        MyFrame_with_simulation = parent_simulation;
        SetBackgroundColour(wxColor(*wxWHITE));
        Bind(wxEVT_PAINT, &Draw_Panel::paintEvent, this);
        Bind(wxEVT_ERASE_BACKGROUND, &Draw_Panel::ignore_EVT_ERASE_BACKGROUND, this);
    }

    /* Does nothing, is used to intercept ignore_EVT_ERASE_BACKGROUND in order to reduce flickering   
    */
    void Draw_Panel::ignore_EVT_ERASE_BACKGROUND(wxEraseEvent& evt) {}

    /*
    * Called by the system of by wxWidgets when the panel needs
    * to be redrawn. You can also trigger this call by
    * calling Refresh()/Update().
    */
    void Draw_Panel::paintEvent(wxPaintEvent& evt) {
        wxPaintDC dc(this);
        render(dc);
    }

    void Draw_Panel::paintNow() {
        wxClientDC dc(this);
        render(dc);
    }

    int Draw_Panel::hexChar_to_num(char c) {
        if ('0' <= c && c <= '9') {
            return c - '0';
        }
        else {
            return c - 'a' + 10;
        }
    }

    wxColor Draw_Panel::hex_to_wxColor(std::string hex) {
        int r = hexChar_to_num(hex[2]) * 16 + hexChar_to_num(hex[3]);
        int g = hexChar_to_num(hex[4]) * 16 + hexChar_to_num(hex[5]);
        int b = hexChar_to_num(hex[6]) * 16 + hexChar_to_num(hex[7]);
        int alpha = hexChar_to_num(hex[8]) * 16 + hexChar_to_num(hex[9]);
        return wxColor(r, g, b, alpha);
    }

    wxPoint Draw_Panel::angle_to_point(int length, int angle_degrees, wxPoint base) {
        int y = base.y + length * - cos(( static_cast<float>(angle_degrees) / 360.0) * 2 * 3.141592);
        int x = base.x + length *   sin(( static_cast<float>(angle_degrees) / 360.0) * 2 * 3.141592);

        return wxPoint(x, y);
    }

    void Draw_Panel::render(wxDC& dc) {

        dc.Clear();

        auto map = MyFrame_with_simulation->simulation.show_map();        
        int horizontal_length = 0;
        int vertical_length = 0;


        dc.SetPen(pen_map_lines);

        //if there even is any map to s how
        if (map.size() > 0) {            

            //calculating scale for current panel size
            scale = std::min((dc.GetSize().y - offset * 2) / (map.size() - 1), (dc.GetSize().x - offset * 2) / (map[0].size() - 1));
            
            horizontal_length = (map[0].size() - 1) * scale;
            vertical_length = (map.size() - 1) * scale;


            //drawing horizontal lines
            for (size_t i = 0; i < map.size(); i++) 
                if (i % 2 == 0)
                    dc.DrawLine(offset, offset + i * scale, offset + horizontal_length, offset + i * scale);
            

            //drawing vertical lines
            for (size_t i = 0; i < map[0].size(); i++)
                if (i % 2 == 0)
                    dc.DrawLine(offset + i * scale, offset, offset + i * scale, offset + vertical_length);
            

            //drawing blanks
            dc.SetBrush(*wxWHITE_BRUSH);
            dc.SetPen(pen_map_obstacles);
            for (size_t i = 0; i < map.size(); i++) 
                for (size_t j = 0; j < map[0].size(); j++) 
                    //if is obstacle
                    if (map[i][j] == '@') 
                        dc.DrawRectangle(offset + (j * scale) - scale / 2, offset + (i * scale) - scale / 2, scale, scale);
                    
                
            
            //Draw agents on theyrs current location (altered_plan)
            pos curr_pos;
            wxPoint curr_center;
            auto agents = MyFrame_with_simulation->simulation.show_agents();

            for (size_t i = 0; i < agents.size(); i++) {

                dc.SetPen(pen_agents);
                curr_pos = agents[i].get_current_position();
                curr_center = wxPoint(curr_pos.x * scale + offset, curr_pos.y * scale + offset);

                //set brush to the color of agent
                dc.SetBrush(hex_to_wxColor(agents[i].get_color()));

                //Draw agent
                dc.DrawCircle(curr_center, agent_radius);

                //Draw error indicating ring around him
                dc.SetBrush(*wxTRANSPARENT_BRUSH);

                if (agents[i].get_error_state() == 0) 
                    //If agent has NO time error  
                    dc.SetPen(pen_error_ring_0);
                else if (agents[i].get_error_state() == 1)
                    //It has small time error                    
                    dc.SetPen(pen_error_ring_1);
                else if (agents[i].get_error_state() == 5)
                    //It is succesfully_moved
                    dc.SetPen(pen_error_ring_5);

                //Circle Around the agent displaying its errorness status
                dc.DrawCircle(curr_center, agent_radius);

                //Draw line indicating agent's current rotation angle
                dc.SetPen(pen_rotation);
                dc.DrawLine(curr_center, angle_to_point( static_cast<int>(agent_radius * 1.5), agents[i].get_rotation(), curr_center));

            }

            //Draw where agents should be (theyrs original plan)
            if (MyFrame_with_simulation->agents_checkboxes_panel) {                
                std::vector<std::string> agent_names_to_draw = MyFrame_with_simulation->agents_checkboxes_panel->get_names_of_checked_agents();

                for (auto& agent : agents) 
                    for (auto& name : agent_names_to_draw)
                        //If I found agent that should be drawn according to where he should be
                        if (name == agent.get_name()) {

                            Agent_move_state mv_state = agent.where_should_I_be(MyFrame_with_simulation->current_time_of_simulation);

                            
                            curr_center = wxPoint(mv_state.current.x * scale + offset, mv_state.current.y * scale + offset);

                            //set brush and pen to the color of agent
                            dc.SetPen(small_pen_agents);
                            dc.SetBrush(hex_to_wxColor(agent.get_color()));

                            //Draw SMALLER agent
                            dc.DrawCircle(curr_center, agent_radius / 2);

                            //Draw SMALLER rotation line
                            dc.SetPen(small_pen_rotation);
                            dc.DrawLine(curr_center, angle_to_point(static_cast<int>(agent_radius * 1.5 / 2), mv_state.rotation, curr_center));

                            //we can go onto next one
                            break;
                        }                                       

            }

        }
    }





/*
*
**** class Agents_Plans_Panel : public wxPanel
*
*/


    /* Only constructor
    */
    Agents_Plans_Panel::Agents_Plans_Panel(wxWindow* parent, MyFrame* Frame_with_simulation) 
    : wxScrolled<wxPanel>(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxFULL_REPAINT_ON_RESIZE),
        Frame_with_simulation(Frame_with_simulation) {

        Agents_Plans_Panel_sizer = new wxBoxSizer(wxVERTICAL);

        Bind_all_scroll_events();
        update_data();
      }
                         
    /* Internally binds all possible scrolling events in order to adequatly show the simulation current time line
    */
    void Agents_Plans_Panel::Bind_all_scroll_events() {

        //It is ugly but as far as I know and forums say, it is the only way

        //Those are all the events that  wxScrolled<wxPanel> is able to emit
        Bind(wxEVT_SCROLLWIN_THUMBTRACK, &Agents_Plans_Panel::scrollPaintEvent, this);
        Bind(wxEVT_SCROLLWIN_THUMBRELEASE, &Agents_Plans_Panel::scrollPaintEvent, this);
        Bind(wxEVT_SCROLLWIN_TOP, &Agents_Plans_Panel::scrollPaintEvent, this);
        Bind(wxEVT_SCROLLWIN_BOTTOM, &Agents_Plans_Panel::scrollPaintEvent, this);
        Bind(wxEVT_SCROLLWIN_LINEDOWN, &Agents_Plans_Panel::scrollPaintEvent, this);
        Bind(wxEVT_SCROLLWIN_LINEUP, &Agents_Plans_Panel::scrollPaintEvent, this);
        Bind(wxEVT_SCROLLWIN_PAGEDOWN, &Agents_Plans_Panel::scrollPaintEvent, this);
        Bind(wxEVT_SCROLLWIN_PAGEUP, &Agents_Plans_Panel::scrollPaintEvent, this);       
    }

    /* Creates grid which is shown to user from the plan
    */
    wxGrid* Agents_Plans_Panel::create_grid_from_plan(const std::vector<plan_step>& plan, std::string label, const std::vector<wxColor>& colors) {

        wxGrid* ret = new wxGrid(this, wxID_ANY);
        ret->CreateGrid(1, plan.size() + 1);
        ret->DisableDragColSize();

        ret->HideRowLabels();
        ret->HideColLabels();
        
        
        ret->ShowScrollbars(wxSHOW_SB_NEVER, wxSHOW_SB_NEVER);
        ret->SetDefaultCellAlignment(wxALIGN_CENTRE, wxALIGN_CENTRE);
        ret->SetDefaultCellFitMode(wxGridFitMode::Overflow()); //Cells Doesn't overflow anyway... strange

        ret->SetCellValue(0, 0, label);
        ret->SetColSize(0, Agent_name_coulmn_width * scaler);
        ret->SetCellBackgroundColour(0, 0, colors[0]);


        for (size_t i = 1; i < plan.size() + 1; i++) {
            auto current_step = plan[i - 1];
            
            //WxWidgets DOES NOT allow columns narrower than 15 if they are not zero ! If you pass number smaller than 15 you get weirdness -> Therefore there can be some inaccuracy ! Even though it's very small one
            if(static_cast<int>(static_cast<float>(current_step.duration) * scaler) == 0)
                ret->SetColSize(i, 0);
            else
                ret->SetColSize(i,std::max(15, static_cast<int>(static_cast<float>(current_step.duration) * scaler))); 

            ret->SetCellValue(0, i, current_step.action);
            // +1 because the colors[0] is the agents color and colors[1] is the color of steps with id == 0;
            ret->SetCellBackgroundColour(0, i, colors[current_step.id + 1]);                        
        }                       
       

        return ret;
    }

    /* Makes and adds wxGrids from plans of agent
    */
    void Agents_Plans_Panel::add_plans_of_agents(const Agent& agent) {

        
        auto original_plan = agent.get_original_plan();
        auto altered_plan = agent.get_altered_plan();

        // index 0 == Agent's color, the rest are colours for plan steps groupped by theyr's id
        std::vector<wxColor> Colours_of_plan_steps;
        srand(time(NULL));

        Colours_of_plan_steps.push_back(Draw_Panel::hex_to_wxColor(agent.get_color()));

        //I can do this because: max id of original == original_plan[original_plan.size() - 1].id == max id of altered AND they start from 0
        for (int i = 0; i <= original_plan[original_plan.size() - 1].id; i++)
            // +156 so the colors dont get too dark
            Colours_of_plan_steps.push_back(wxColor(((rand() % 256) + 156) % 256, ((rand() % 256) + 156) % 256, ((rand() % 256) + 156) % 256));

        


        plans_of_agents.push_back(create_grid_from_plan(original_plan, agent.get_name() + " Original", Colours_of_plan_steps));
        plans_of_agents.push_back(create_grid_from_plan(altered_plan , agent.get_name() + " Altered" , Colours_of_plan_steps));

    }

    /* if simulation is empty, it doesn't show anything
    * It loads all the data from scratch
    */
    void Agents_Plans_Panel::update_data() {

        //Clear all of the old data
            //This handles destruction of wxGrids
        Agents_Plans_Panel_sizer->Clear(true);
            //This resets the array
        plans_of_agents.clear();

        //Build new data
        auto const_agents = Frame_with_simulation->simulation.show_agents();

        for (const Agent& agent : const_agents)
            add_plans_of_agents(agent);

        //Add them to sizer
        for (size_t i = 0; i < plans_of_agents.size(); i++) {
            Agents_Plans_Panel_sizer->Add(plans_of_agents[i]/*, wxEXPAND*/);

            if(i % 2)
                Agents_Plans_Panel_sizer->AddSpacer(20);
        }
            

        this->SetSizer(Agents_Plans_Panel_sizer);        
        this->FitInside(); // ask the sizer about the needed size
        this->SetScrollRate(5, 5);

        update_simulation_current_time_line();
    }

    /* May be called by system
    */
    void Agents_Plans_Panel::paintEvent(wxPaintEvent& evt) {

        wxPaintDC dc(this);
        render(dc);    
    }


    /* May be called by the system due to scrolling
    */
    void Agents_Plans_Panel::scrollPaintEvent(wxScrollWinEvent& event) {

        update_simulation_current_time_line();
        event.Skip(true);
    }

    /* May be called manually to update the simulation current time line.
    */
    void Agents_Plans_Panel::update_simulation_current_time_line() {

        wxClientDC dc(this);
        render(dc);
    }

    /* Draws the simulation current time line
    * Cannot be drawn on the wxGrids due to the wxWidgets limitations
    */
    void Agents_Plans_Panel::render(wxDC& dc) {

        DoPrepareDC(dc);
        dc.Clear();
        
        dc.SetPen(wxPen(wxColor(0, 0, 0), 10));

        int x_offset = Agent_name_coulmn_width * scaler;
        int x_for_line = x_offset + Frame_with_simulation->current_time_of_simulation * scaler;

        int y_height = dc.GetSize().y;

        dc.DrawLine(x_for_line, 0, x_for_line, y_height);
    }




/*
*
**** class Agents_CheckBoxes_Panel : public wxPanel
*
*/

    Agents_CheckBoxes_Panel::Agents_CheckBoxes_Panel(wxWindow* parent, MyFrame* Frame_with_simulation, wxWindowID id, wxSize size) :
        wxPanel(parent, id, wxDefaultPosition, size),
        Frame_with_simulation(Frame_with_simulation) {
        
        Agents_CheckBoxes_sizer = new wxBoxSizer(wxVERTICAL);
        reload_data();
        Bind(wxEVT_CHECKBOX, &Agents_CheckBoxes_Panel::on_checkbox_click, this);        
    }

    void  Agents_CheckBoxes_Panel::on_checkbox_click(wxCommandEvent& event) {
        //Just redraw the simulation so the user can see the results of his actions
        Frame_with_simulation->panel_simulation->paintNow();
    }

    void Agents_CheckBoxes_Panel::reload_data() {

        if (!Frame_with_simulation)
            return;

        //Clear all of the old data
            //This handles destruction of CheckBoxes and texts
        Agents_CheckBoxes_sizer->Clear(true);


        //This resets the vectors
        checkboxes.clear();

        //Build new data
        auto agents = Frame_with_simulation->simulation.show_agents();
        for (size_t i = 0; i < agents.size(); i++) {

            //Create Agents checkBoxes with text as theyr names having theyr color
            checkboxes.push_back(new wxCheckBox(this, wxID_ANY, agents[i].get_name()));
            checkboxes[i]->SetForegroundColour(Draw_Panel::hex_to_wxColor(agents[i].get_color()));            

            Agents_CheckBoxes_sizer->AddSpacer(10);
            Agents_CheckBoxes_sizer->Add(checkboxes[i]);
        }

        SetSizerAndFit(Agents_CheckBoxes_sizer);

        Layout();       
    }

    std::vector<std::string> Agents_CheckBoxes_Panel::get_names_of_checked_agents() {
        std::vector<std::string> ret;

        for(auto checkbox : checkboxes)
            if (checkbox->IsChecked())
                ret.push_back(checkbox->GetLabelText().ToStdString());
        
        return ret;
    }









    /*
    *
    **** class Collision_detection_Dialog : public wxDialog
    *
    */



 

    //public:

    Collision_detection_Dialog::Collision_detection_Dialog() 
        : wxDialog(nullptr, wxID_ANY, "Choose CDM") {

        chosen_detection_method = "";

        Collision_detection_sizer = new wxBoxSizer(wxVERTICAL);

        txt_dialogs_title = new wxStaticText(this, wxID_ANY, "Choose which collision detection method you want to be used.");
        Collision_detection_sizer->Add(txt_dialogs_title, 0, wxALL, 10);
        Collision_detection_sizer->AddSpacer(20);



        wxArrayString wx_method_names;

        wx_method_names.Add("None");
        wx_method_names.Add("Line_Detection");
        wx_method_names.Add("Variable_Sampling_Detection");
        wx_method_names.Add("Static_Sampling_Detection");
        wx_method_names.Add("Rectangle_Detection");
        
        list_box = new wxListBox(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wx_method_names);
        Collision_detection_sizer->Add(list_box, 2, wxEXPAND | wxALL, 10);
        Collision_detection_sizer->AddSpacer(20);

        button_set = new wxButton(this, wxID_OK, "Set", wxDefaultPosition, wxSize(150, 50));
        Collision_detection_sizer->Add(button_set, 2, wxEXPAND | wxALL, 10);

        SetSizerAndFit(Collision_detection_sizer);
    }

    std::string Collision_detection_Dialog::get_data() {

        if (list_box->GetSelection() == -1)
            return DEFAULT_DETECTION_METHOD;

        return list_box->GetString(list_box->GetSelection()).ToStdString();
    }
    