#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <sstream>
#include <stdlib.h> 

struct pos {
    float x;
    float y;
};

class plan_step {
public:
    int duration;
    pos position;
    int rotation;
    std::string action;


    plan_step(pos position, int rotation, std::string action, int duration) :
        position(position), rotation(rotation), action(action), duration(duration) {
    }
};

class Time_diff_error {
public:
    
    int  time_stamp; //Time when agent has time difference against original plan of size time_stamp
    int   time_diff; //time in miliseconds agent is behind or forward (sign depended

    Time_diff_error(/*int agent_index,*/ int time_stamp, int time_diff) : /*agent_index(agent_index),*/ time_stamp(time_stamp), time_diff(time_diff) { }

    bool operator < (const Time_diff_error& second) const
    {
        return (time_stamp < second.time_stamp); //For sorting
    }
};

class Agent {
    std::string name; //id
    std::string color;
    pos start;
    pos finish;
    pos current;
    int rotation = 0;


    int altered_max_time = 0;

    int agent_error_speed_max = 0;
    int agent_error_speed_sigma = 0;

    int agent_error_angle_max = 0;
    int agent_error_angle_sigma = 0;
    int agent_error_angle_fatal = 0;

    std::vector<plan_step> original_plan;
    std::vector<plan_step> altered_plan;

    bool errorness = false;//If the agent is late or soon. In other words it has bad timing
public:


    Agent(std::string name, std::string color, pos start, pos finish, const std::vector<int>& errors) : //errors:  0 = speed max, 1 = speed sigma, 2 = angle max, 3 = angle sigma, fatal angle
        name(name), color(color), start(start), finish(finish) {
        agent_error_speed_max = errors[0];
        agent_error_speed_sigma = errors[1];

        agent_error_angle_max = errors[2];
        agent_error_angle_sigma = errors[3];
        agent_error_angle_fatal = errors[4];

        current = start;
    }

    //moves agent to absolute time
    void move_to_time(int time) {//time in miliseconds    

        if (time > altered_max_time) { //If I was supposed to move more than I can i stand at the end of my plan

            current = altered_plan[altered_plan.size() - 1].position;
            return;
        }

        int tmp_time = 0;
        for (size_t i = 0; i < altered_plan.size(); i++) {
            if (tmp_time + altered_plan[i].duration < time) { //If I am supposed to start from following plan_step and not this
                tmp_time += altered_plan[i].duration;
            }
            else {//movement started from i-th plan_step

                if (altered_plan[i].action == "start" || altered_plan[i].action == "end" || altered_plan[i].action == "wait") {
                    current = altered_plan[i].position; //Go to the starting position of this plan_step

                    rotation = altered_plan[i].rotation;
                }
                else { //Else I move to the location between the 2 plan_steps
                    current.x = altered_plan[i].position.x + (static_cast<float>((time - tmp_time)) / static_cast<float>(altered_plan[i].duration)) * (altered_plan[i + 1].position.x - altered_plan[i].position.x); //There is always i+1 th action because if i-th action was end, i didnt get into this else branch
                    current.y = altered_plan[i].position.y + (static_cast<float>((time - tmp_time)) / static_cast<float>(altered_plan[i].duration)) * (altered_plan[i + 1].position.y - altered_plan[i].position.y); //

                    rotation = altered_plan[i].rotation + (static_cast<float>((time - tmp_time)) / static_cast<float>(altered_plan[i].duration)) * (altered_plan[i + 1].rotation - altered_plan[i].rotation);
                }
                break;
            }
        }
    }

    std::string get_color() {
        return color;
    }

    std::string get_name() {
        return name;
    }

    pos get_current_position() {
        return current;
    }

    int get_rotation() {
        return rotation;
    }

    auto get_altered_plan_length() {
        return altered_max_time;
    }

    auto get_altered_plan() {
        auto ret = altered_plan;
        return ret;
    }

    auto get_err_vector() {
        std::vector<int> err = { agent_error_speed_max, agent_error_speed_sigma, agent_error_angle_max, agent_error_angle_sigma, agent_error_angle_fatal };
        return err;
    }

    auto get_original_plan() {
        auto ret = original_plan;
        return ret;
    }

    void set_error_state(bool b) {
        errorness = b;
    }

    bool get_error_state() {
        return errorness;
    }



    void set_errors(const std::vector<int>& errors) {
        agent_error_speed_max = errors[0];
        agent_error_speed_sigma = errors[1];

        agent_error_angle_max = errors[2];
        agent_error_angle_sigma = errors[3];
        agent_error_angle_fatal = errors[4];
    }

    void set_original_plan(std::vector<plan_step>&& vct) {
        original_plan = std::move(vct);
    }

    void set_altered_plan(std::vector<plan_step>&& vct) {
        altered_plan = std::move(vct);
        altered_max_time = 0;
        for (size_t i = 0; i < altered_plan.size(); i++)
            altered_max_time += altered_plan[i].duration;
    }

    void set_altered_plan(std::vector<plan_step>&& vct, int total_altered_plan) {
        altered_plan = std::move(vct);
        altered_max_time = total_altered_plan;
    }

};


class Simulation {
    std::vector<std::vector<char>> map;
    std::vector<Agent> agents;
    std::vector<std::vector<Time_diff_error>> time_diffs_of_agents;


    const int ERROR_TRESHOLD = 1000; /////////// ERROR IS CONSIDERED ONLY IF AGENT IS LATE OR SOONER THEN 1 SECONDs
    int agent_plan_max_length = 0;

    float normal_distribution(float sigma, float x) { //u == 0 always

        return (1 / (sigma * sqrtf(2 * 3.14159))) * pow(2.71828, -0.5 * pow(x / sigma, 2));

    }

    void alter_plans_of_agents(int from, int to) { //weird indexes imply that this function can be used to alter one or all of the agents plans ///////////////////////////////////////// IS IGNORING TURNING ANGLES NEED TO FINISH
        int tmp_error_speed_max = 0;
        int tmp_error_speed_sigma = 0;

        int tmp_error_angle_max = 0;
        int tmp_error_angle_sigma = 0;
        int tmp_error_angle_fatal = 0;

        std::vector<plan_step> altered;
        std::vector<plan_step> original;

        std::vector<int> err_vec;
        std::vector<std::pair<int, float>> prob_map;// """pigeon hole principle"""
        std::pair<int, float> prob_pair;

        int prob_sum;

        const float scaler = 100; //for the normal distributions
        const float precision = 1.0;

        int value = 0;
        int add_up = 0;
        int idx_of_rolled_pair = 0;

        int time_counter_original = 0;
        int time_counter_altered = 0;




        for (size_t i = from; i < to; i++) { //for every agent
            err_vec = agents[i].get_err_vector();
            altered = agents[i].get_original_plan();
            tmp_error_speed_max = err_vec[0];
            tmp_error_speed_sigma = err_vec[1];

            tmp_error_angle_max = err_vec[2];
            tmp_error_angle_sigma = err_vec[3];
            tmp_error_angle_fatal = err_vec[4];

            prob_sum = 0;
            for (float x = 0; x < tmp_error_speed_max; x += precision) { //create the table of probabilities
                //first is probablity * scaler casted into int
                //second is value of x
                prob_pair = std::make_pair(static_cast<int>(normal_distribution(tmp_error_speed_sigma, x) * scaler), x);
                prob_sum += prob_pair.first;
                prob_map.push_back(prob_pair);
                if (x != 0) {
                    prob_sum += prob_pair.first;
                    prob_pair.second = prob_pair.second * (-1); //normal distribution is symetrical when u==0
                    prob_map.push_back(prob_pair);
                }
            }

            if (tmp_error_speed_max != 0) {//If theres no room to change speed, just dont
                for (size_t j = 0; j < altered.size(); j++) { //Alter the duration of plan_steps to create altered plan
                    //for every step just roll the dices
                    srand(i * j * 123 + 321);
                    value = rand() % prob_sum;
                    add_up = 0;

                    //Find the pair of rolled x
                    for (size_t i = 0; i < prob_map.size(); i++) {
                        add_up += prob_map[i].first;
                        if (add_up >= value) {//I rolled this pair
                            idx_of_rolled_pair = i;
                            break;
                        }
                    }

                    altered[j].duration = altered[j].duration + prob_map[idx_of_rolled_pair].second * 1000; //alter the duration ____________________ *1000 to delete????//////////////////////////

                }
            }


            //Note-ing time differences of original vs altered plan of agent, so I can detect errors later                    
            time_diffs_of_agents[i].clear(); //Reset it for pushabcking new items

            time_counter_altered = 0;
            time_counter_original = 0;
            original = agents[i].get_original_plan();
            for (size_t step = 0; step < altered.size(); step++) { //Errors can be noted ONLY when the agent finishes the action. Motivation is that he could make up for his mistake until the end of the plan_step action
                time_counter_original += original[step].duration;
                time_counter_altered += altered[step].duration;

                time_diffs_of_agents[i].emplace_back(Time_diff_error(time_counter_altered, time_counter_altered - time_counter_original));
            }


            agents[i].set_altered_plan(std::move(altered), time_counter_altered);
            altered.clear();
            //And onto the next Agent!
        }
    }

    /* updates agent_plan_max_length.
    * Goes through all agents and compares theyrs plan_length to agent_plan_max_length
    */
    void update_agent_plan_max_length() {
        agent_plan_max_length = 0;

        for (auto& agent : agents)
            if (agent.get_altered_plan_length() > agent_plan_max_length)
                agent_plan_max_length = agent.get_altered_plan_length();
    }

    /* Returns index to specified agent's timediff field relevant for time
    *  if returns -1, the agent havent even finished his 1st plan_step and therefore it cannot be late/soon
    */
    int get_index_of_timediff_for_agent_at_time(int agents_index, int time) {
        int idx = -1;
        for (size_t j = 0; j < time_diffs_of_agents[agents_index].size(); j++) {
            if (time_diffs_of_agents[agents_index][j].time_stamp <= time) //If I have already finished j-th step
                idx = j; //Remember it
            else
                break;
        }

        return idx;
    }

    /* 
    * Puts the concrete Solver Option Predicat Name into the line
    */
    void load_plans_get_SolverOptionPredicatName(std::ifstream& myfile, std::string& line) {
        while (std::getline(myfile, line)) {
            if (line == "#SolverOptionPredicatName") { //The line before the 
                std::getline(myfile, line);
                return;
            }
        }

        //shouldn't get here
        return;
    }

    /*
    * Puts the firsts agent definition into the line
    */
    void load_plans_get_FirstAgentDefinition(std::ifstream& myfile, std::string& line) {    
        while (std::getline(myfile, line)) {
            if (line == "#Agents: AgentNO AgentName AgentColor (startX,startY) (endX,endY)") { //The line before the agents definitions
                std::getline(myfile, line); //to get the firsts agent definition line
                break;
            }
        }

        //shouldn't get here
        return;
    }

    void load_plans_load_all_agents(std::ifstream& myfile, std::string& line, std::vector<Agent>& agents_vector, const std::vector<int>& errors_vectors, const std::string& delimiter) {

        load_plans_get_FirstAgentDefinition(myfile, line);
        //now I have 1st agent definition in line

        size_t idx = 0;
        std::string agt_name;
        std::string agt_color;
        pos agt_start;
        pos agt_finish;

        while (line != "") { //There is a blank line after the agents definitions

            idx = line.find(delimiter);
            line.erase(0, idx + delimiter.length()); //I dont care about "IDs"

            idx = line.find(delimiter);
            agt_name = line.substr(0, idx); //name of agent
            line.erase(0, idx + delimiter.length());

            idx = line.find(delimiter);
            agt_color = line.substr(0, idx); //color of agent
            line.erase(0, idx + delimiter.length());


            agt_start.x = std::stof(&line[1]);
            idx = line.find(',');
            agt_start.y = std::stof(&line[idx + 1]);

            idx = line.find(delimiter);
            line.erase(0, idx + delimiter.length());

            agt_finish.x = std::stof(&line[1]);
            idx = line.find(',');
            agt_finish.y = std::stof(&line[idx + 1]);

            agents_vector.emplace_back(Agent(agt_name, agt_color, agt_start, agt_finish, errors_vectors));


            std::getline(myfile, line);
        }
    }

    /*
    * Loads the map
    */
    void load_plans_load_map(std::ifstream& myfile, std::string& line, std::vector<std::vector<char>>& map) {

        size_t map_x = 0;
        size_t map_y = 0;

        while(std::getline(myfile, line)) {
            if (line == "#MapSizeX,MapSizeY") {
                std::getline(myfile, line); //get the data

                map_x = std::stoi(&line[1]) * 2 - 1; //mapsize X
                map_y = std::stoi(&line[line.find(',') + 1]) * 2 - 1; //mapsize Y
                
                break;
            }
        }
        //now I have the correct map sizes


        //Set the map size accordingly
        map.resize(map_y);
        for (size_t i = 0; i < map_y; i++)
            map[i].resize(map_x, '.'); // '.' means travesible


        //Set untraversible spots which are there because the map is a grid
        for (size_t y = 0; y < map_y; y++)
            for (size_t x = 0; x < map_x; x++)
                if (y % 2 == 1 && x % 2 == 1)
                    map[y][x] = '@'; //Due to the map format in use



        //Now load the untraversible spots which are there because the map has obstacles
        std::getline(myfile, line); //
        std::getline(myfile, line); // Skip the boring lines

        while (line != "") { //while I am loading obstacles
            map_x = std::stoi(&line[1]);
            map_y = std::stoi(&line[line.find(',') + 1]);

            map[map_y][map_x] = '@'; //'@' stands for obstacle

            std::getline(myfile, line);
        }
    }

    void load_plans_load_num_to_vertex_map(std::ifstream& myfile, std::string& line, std::map<int, pos>& num_to_vertex) {

        //Get to the first usefull line
        while (std::getline(myfile, line))
            if (line == "#PicatIdToVertexNum: VertexNum->(PositionX,PositionY)") {
                std::getline(myfile, line);
                break;
            }

        //load all mappings
        pos act;
        while (line != "#AgentNO VertexNo Rotation Action Duration SomeText ") {
            act.x = std::stof(&line[line.find('(') + 1]);
            act.y = std::stof(&line[line.find(',') + 1]);
            num_to_vertex.emplace(std::stoi(&line[0]), act);

            std::getline(myfile, line);
        }
    }


    /*
    * plan_step_durations[0] == go_duration, plan_step_durations[1] == turn_duration, plan_step_durations[2] == wait_duration
    */
    void load_plan_load_and_normalize_agents_plans(std::ifstream& myfile, std::string& line, std::map<int, pos>& num_to_vertex, 
        std::vector<Agent>& agents_vector, const std::string& solver_name, const std::vector<int>& plan_step_durations) {

        size_t agt_idx = 0;
        std::string token;

        pos agt_position;
        int agt_rotation;
        int agt_duration;
        std::string agt_action;
        
        std::vector<plan_step> agt_plan;


        while (std::getline(myfile, line)) { //loading plans

            //If I have finished loading the agent
            if (line == "") {
                agents_vector[agt_idx].set_original_plan(std::move(agt_plan)); //Give agent its plan
                agt_plan.clear(); //Get it ready for another round

                ++agt_idx;
                continue;
            }

            std::istringstream iss(line);

            //Skip AgentNO
            std::getline(iss, token, ' '); 


            //Get the vertex num
            std::getline(iss, token, ' '); 
            agt_position = num_to_vertex[std::stoi(token)];


            // Get rotation
            std::getline(iss, token, ' ');
            if (token == "null")
                agt_rotation = 0;
            else
                agt_rotation = std::stoi(token);


            // Get action
            std::getline(iss, agt_action, ' ');


            // Get duration
            std::getline(iss, token, ' ');
            agt_duration = std::stoi(token);


            //And I ignore "SomeText"


            auto normalized_plan_steps = get_normalized_plan_steps_from(plan_step(agt_position, agt_rotation, agt_action, agt_duration), solver_name, agt_plan, plan_step_durations);

            //Merge them
            for (size_t i = 0; i < normalized_plan_steps.size(); i++)
                agt_plan.push_back(std::move(normalized_plan_steps[i]));
                        
        }
    }

    /* Retuns vector of normalized plan_steps from plan_step step
    * plan_step_durations[0] == go_duration, plan_step_durations[1] == turn_duration, plan_step_durations[2] == wait_duration
    */
    std::vector<plan_step> get_normalized_plan_steps_from(plan_step&& step, const std::string& solver_name, const std::vector<plan_step>& preceding_normalized_agent_plan_step, const std::vector<int>& plan_step_durations) {

        std::vector<plan_step> ret;

        const int go_duration = plan_step_durations[0];
        const int turn_duration = plan_step_durations[1];
        const int wait_duration = plan_step_durations[2];

        if (solver_name == "mapf_edge_split_rob" ||
            solver_name == "mapf_edge_split" ||
            solver_name == "mapf_edge_disjoint") {
            //in this case the plan doesn't use rotation field nor the turn instruction
            //So I have to add them


            if (step.action == "start") {
                //no need to change anything except of duration
                //here step.rotation == 0 always
                ret.emplace_back(plan_step(step.position, step.rotation, "start", wait_duration));
                return ret;

            }

            //so I can get the angle the agent was previosly to this step
            //would preceeding[-1] only when when I would be reading the first (start) plan_instruction, but when that is the case I wont get here so it is Ok
            plan_step previous_step = preceding_normalized_agent_plan_step[preceding_normalized_agent_plan_step.size() - 1];


            if (step.action == "end") {
                //need to rotate to 0 rotation angle
                if (previous_step.rotation == 90) {
                    //it will be faster to turnLeft
                    ret.emplace_back(plan_step(previous_step.position, previous_step.rotation, "turnLeft", turn_duration));
                }
                else {
                    //it will be faster or equaly as fast to turnRight
                    for (size_t i = 0; ((previous_step.rotation + i) % 360) != 0 ; i += 90) 
                        ret.emplace_back(plan_step(previous_step.position, (previous_step.rotation + i) % 360, "turnRight", turn_duration));
                }


                ret.emplace_back(plan_step(previous_step.position, 0, "end", wait_duration));
                return ret;

            }
            else if (step.action == "waitB") {
                //need to change the name to wait and correct the rotation field and duration
                ret.emplace_back(plan_step(step.position, previous_step.rotation, "wait", wait_duration));
                return ret;

            }
            else if (step.action == "goB") {
                //change action name from goB to go and correct the rotation field and duration
                ret.emplace_back(plan_step(step.position, previous_step.rotation, "go", go_duration));
                return ret;

            }
            else if (step.action == "leftGo") {
                //need to split into turnLeft and go instruction
                ret.emplace_back(plan_step(step.position, previous_step.rotation, "turnLeft", turn_duration));
                ret.emplace_back(plan_step(step.position, (previous_step.rotation + 270) % 360, "go", go_duration));
                return ret;

            }
            else if (step.action == "rightGo") {
                //need to split into turnRight and go instruction
                ret.emplace_back(plan_step(step.position, previous_step.rotation, "turnRight", turn_duration));
                ret.emplace_back(plan_step(step.position, (previous_step.rotation + 90) % 360, "go", go_duration));
                return ret;

            }
            else if (step.action == "backwardGo") {
                //need to split into 2x turnRight and go instruction
                ret.emplace_back(plan_step(step.position, previous_step.rotation, "turnRight", turn_duration));
                ret.emplace_back(plan_step(step.position, (previous_step.rotation + 90) % 360, "turnRight", turn_duration));
                ret.emplace_back(plan_step(step.position, (previous_step.rotation + 180) % 360, "go", go_duration));
                return ret;

            }
            

        }
        else if (solver_name == "mapf_vertex_split_rob" ||
            solver_name == "mapf_vertex_split") {
            //in this case the plan already has only rotation, go, start, end, wait instructions
            //Also it's already using Rotation field correctly

             if (step.action == "start") {
                 //here step.rotation == 0 always
                 ret.emplace_back(plan_step(step.position, step.rotation, "start", wait_duration));
                 return ret;

             }
             else if (step.action == "end") {
                 //no need to change anything except of duration
                 ret.emplace_back(plan_step(step.position, step.rotation, "end", wait_duration));
                 return ret;

             }
             else if (step.action == "waitC") {
                 //change action name from waitC to wait and duration                
                 ret.emplace_back(plan_step(step.position, step.rotation, "wait", wait_duration));
                 return ret;

             }
             else if (step.action == "goC") {
                 //change action name from goC to go and duration
                 ret.emplace_back(plan_step(step.position, step.rotation, "go", go_duration));
                 return ret;

             }
             else if (step.action == "turnLeft") {
                 //no need to change anything except of duration
                 ret.emplace_back(plan_step(step.position, step.rotation, "turnLeft", turn_duration));
                 return ret;

             }
             else if (step.action == "turnRight") {
                 //no need to change anything except of duration
                 ret.emplace_back(plan_step(step.position, step.rotation, "turnRight", turn_duration));
                 return ret;

             }
        }
        else if (solver_name == "mapf_simple") {
            //in this case the plan doesn't use rotation field nor the turn instruction
            //So I have to add them

            if (step.action == "start") {
                //here step.rotation == 0 always
                ret.emplace_back(plan_step(step.position, step.rotation, "start", wait_duration));
                return ret;
            }
            else if (step.action == "end") {
                //need to rotate to 0 rotation angle
                plan_step previous_step = preceding_normalized_agent_plan_step[preceding_normalized_agent_plan_step.size() - 1];

                if (previous_step.rotation == 90) {
                    //it will be faster to turnLeft
                    ret.emplace_back(plan_step(previous_step.position, previous_step.rotation, "turnLeft", turn_duration));
                }
                else {
                    //it will be faster or equaly as fast to turnRight
                    for (size_t i = 0; ((previous_step.rotation + i) % 360) != 0; i += 90)
                        ret.emplace_back(plan_step(previous_step.position, (previous_step.rotation + i) % 360, "turnRight", turn_duration));
                }


                ret.emplace_back(plan_step(previous_step.position, 0, "end", wait_duration));
                return ret;

            }
            else if (step.action == "wait") {
                plan_step previous_step = preceding_normalized_agent_plan_step[preceding_normalized_agent_plan_step.size() - 1];
                ret.emplace_back(plan_step(step.position, previous_step.rotation, "wait", wait_duration));
                return ret;

            }
            else if (step.action == "north" ||
                 step.action == "east" ||
                 step.action == "south" ||
                 step.action == "west") {
                 //make rotate and go

                 //so I can get the angle the agent was previosly to this step
                 //would preceeding[-1] only when when I would be reading the first (start) plan_instruction, but when that is the case I wont get here so it is Ok
                 plan_step previous_step = preceding_normalized_agent_plan_step[preceding_normalized_agent_plan_step.size() - 1];

                 int angle_delta = previous_step.rotation - cardinal_to_angle(step.action);

                 //rotate to the right angle
                 if (angle_delta == 90) {
                     ret.emplace_back(plan_step(step.position, previous_step.rotation, "turnLeft", turn_duration));
                 }
                 else if (angle_delta == -90) {
                     ret.emplace_back(plan_step(step.position, previous_step.rotation, "turnRight", turn_duration));
                 }
                 else if (std::abs(angle_delta) == 180) {
                     ret.emplace_back(plan_step(step.position, previous_step.rotation, "turnRight", turn_duration));
                     ret.emplace_back(plan_step(step.position, (previous_step.rotation + 90) % 360, "turnRight", turn_duration));
                 }

                 ret.emplace_back(plan_step(step.position, cardinal_to_angle(step.action), "go", go_duration));
                 return ret;
            }          
        }



        //default, shouldn't happen
        return ret;
    }

    int cardinal_to_angle(const std::string& str) {
        if (str == "north") return 0;
        else if (str == "east") return 90;
        else if (str == "south") return 180;
        else if (str == "west") return 270;

        return 0;
    }


public:

    Simulation() {}


    /*
    * plan_step_durations[0] == go_duration, plan_step_durations[1] == turn_duration, plan_step_durations[2] == wait_duration
    */
    void load_plans(std::string path, const std::vector<int>& plan_step_durations) {

        //load vector of errors  
        std::vector<int> err = { 0, 0, 0, 0, 0 };               

        //Reseting Stuff
        agents.clear();
        map.clear();
        time_diffs_of_agents.clear();
        agent_plan_max_length = 0;


        std::string line;
        std::ifstream myfile;
    
        std::string delimiter = " ";
        std::string solver_option_predicat_name;



        myfile.open(path);
        if (myfile.is_open()) {

            load_plans_get_SolverOptionPredicatName(myfile, line);
            solver_option_predicat_name = line;

            load_plans_load_all_agents(myfile, line, agents, err, delimiter);
            //Now I have loaded all the agents
            
            load_plans_load_map(myfile, line, map);
            //Now I have my map loaded
            
            std::map<int, pos> num_to_vertex;
            load_plans_load_num_to_vertex_map(myfile, line, num_to_vertex);
            //now I have map from vertices IDs to theyr real positions


            //And finally it is time to load and normalize agents plans
            load_plan_load_and_normalize_agents_plans(myfile, line, num_to_vertex, agents, solver_option_predicat_name, plan_step_durations);

            myfile.close();
        }
        else {

            std::cout << "Cannot find file " + path;
        }



        if (time_diffs_of_agents.size() != agents.size()) //Time diffs always have the right size
            time_diffs_of_agents.resize(agents.size());

        alter_plans_of_agents(0, agents.size());

        update_agent_plan_max_length();
    }

    /* moves all agents of simulation to specified time
    * Returns true if At least one agent moved. When it returns False, there is no point in moving to bigger time.
    * if time < 0 it moves to time == 0 and returns false
    */
    bool move_to_time(int time) {

        if (time < 0) {
            //Move to the closest possible state -> zero
            move_to_time(0);

            //But return false anyway
            return false;
        }


        int idx = 0;

        for (size_t i = 0; i < agents.size(); i++) {
            agents[i].move_to_time(time);



            //Check if the time difference for agent[i] at time 'time' is too big

            idx = get_index_of_timediff_for_agent_at_time(i, time);
            //Now in the idx is the index of the last step I have finished (-1 == I havent finished even the first one)

            if (idx == -1) //If I havent even finished first step (index 0), I cannot be too late/soon
                agents[i].set_error_state(false);
            else if (std::abs(time_diffs_of_agents[i][idx].time_diff) >= ERROR_TRESHOLD) //If I am too late/soon
                agents[i].set_error_state(true);
            else
                agents[i].set_error_state(false); //Otherwise I am ok


        }

        if (time > agent_plan_max_length) {
            return false;
        }
        else {
            return true;
        }
    }




    std::vector<std::string> get_agents_names() {
        std::vector<std::string> ret;

        for (size_t i = 0; i < agents.size(); i++) {
            ret.push_back(agents[i].get_name());
        }
        return ret;
    }

    /* Sets Agent's errors and also alters his plan.
    * Also updates agent_plan_max_length so it stays correct
    */
    void set_errors_to_agent(std::string& agent_name, const std::vector<int>& parameters) { // 0 = speed max, 1 = speed sigma, 2 = angle max, 3 = angle sigma,4 = fatal angle

        for (size_t i = 0; i < agents.size(); i++) {
            if (agents[i].get_name() == agent_name) {
                agents[i].set_errors(parameters);
                alter_plans_of_agents(i, i + 1);

                update_agent_plan_max_length();
                break;
            }
        }

    }

    /*
    * returns how late/soon the agent is at specified time
    */
    int get_time_diffs_of_agent_at_time(int agent_index, int time) {

        int timediff_idx = get_index_of_timediff_for_agent_at_time(agent_index, time);

        if (timediff_idx == -1) {
            return 0;
        }
        else
            return time_diffs_of_agents[agent_index][timediff_idx].time_diff;
    }

    /*
    * Gives only const reference
    */
    const std::vector<std::vector<char>>& show_map() {        
        return map;
    }

    /*
    * Gives only const reference
    */
    const std::vector<Agent>& show_agents() {
        return agents;
    }

    auto get_agent_plan_max_length() {
        return agent_plan_max_length;
    }














    /*void load_global_errors(std::vector<int>&& vct) {// 0 = speed max, 1 = speed sigma, 2 = angle max, 3 = angle sigma,4 = fatal angle
    error_speed_max = vct[0];
    error_speed_sigma = vct[1];

    error_angle_max = vct[2];
    error_angle_sigma = vct[3];
    error_angle_fatal = vct[4];

    for (size_t i = 0; i < agents.size(); i++) {
        agents[i].set_errors(vct);
    }

    alter_plans_of_agents(0, agents.size());

    agent_plan_max_length = 0;
    for (size_t i = 0; i < agents.size(); i++)
        if (agents[i].get_altered_plan_length() > agent_plan_max_length)
            agent_plan_max_length = agents[i].get_altered_plan_length(); //Update agent_plan_max_length
}*/

/*void load_map(std::string path) {
std::ifstream myfile;
std::string line;
myfile.open(path);

if (myfile.is_open()) {
    std::getline(myfile, line);//skip useless line

    std::getline(myfile, line);
    int i = 0;
    for (; line[i] != ' '; ++i) {} ++i; //get first height digit index
    int height = std::atoi(&line[i]);

    std::getline(myfile, line);
    i = 0;
    for (; line[i] != ' '; ++i) {} ++i; //get first width digit index
    int width = std::atoi(&line[i]);

    i = 0;
    std::getline(myfile, line);//skip useless line


    map.resize(height);
    for (size_t y = 0; y < height; y++)
    {
        map[y].resize(width);
        std::getline(myfile, line);
        for (size_t x = 0; x < width; x++) {
            map[y][x] = line[x];
        }

    }
    myfile.close();
}
else {
    std::cout << path << " NOT FOUND" << std::endl;
}

}*/

/*
 void load_agents(std::string path) {
    std::vector<int> err; err.push_back(error_speed_max); err.push_back(error_speed_sigma); err.push_back(error_angle_max); err.push_back(error_angle_sigma); err.push_back(error_angle_fatal); //load vector of errors

    std::ifstream myfile;
    std::string line;
    myfile.open(path);
    if (myfile.is_open()) {

        std::getline(myfile, line); //read the first line, where the number of agents is stored
        int n_agents = std::stoi(line);

        for (size_t i = 0; i < n_agents; ++i) {
            std::getline(myfile, line); //load the line where are the data of current agent
            std::string curr_id = "";
            std::string curr_color = "";
            pos curr_start;
            pos curr_finish;

            int which = 0; //which thing I am loading at the moment (if name or color or position etc)
            bool end = false;
            for (size_t idx = 0; !end; idx++) { // for every char in line
                if (which == 0) { //loading id
                    if (line[idx] != ';')
                        curr_id += line[idx];
                    else
                        ++which;
                }
                else if (which == 1) {//loading color
                    if (line[idx] != ';')
                        curr_color += line[idx];
                    else
                        ++which;
                }
                else if (which == 2) {//loading coords
                    ++idx; // skip the unwanted '('
                    curr_start.x = std::stof(&line[idx]); //load start coord x
                    idx += (std::to_string(curr_start.x)).size() + 1;//set idx onto the begining of start coord y

                    curr_start.y = std::stof(&line[idx]); //load start coord y
                    idx += (std::to_string(curr_start.y)).size() + 3;//set idx onto the beginning of finish coord x



                    curr_finish.x = std::stof(&line[idx]); //load finish coord x
                    idx += (std::to_string(curr_finish.x)).size() + 1;//set idx onto the begining of finish coord y

                    curr_finish.y = std::stof(&line[idx]); //load finish coord y
                    end = true;
                }
            }

            Agent ret(curr_id, curr_color, curr_start, curr_finish, err);
            agents.push_back(ret);
        }
        myfile.close();
    }
    else {
        std::cout << path << " NOT FOUND" << std::endl;
    }


}
*/

};