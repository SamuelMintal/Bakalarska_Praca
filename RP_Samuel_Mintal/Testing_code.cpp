#include "Testing_code.hpp"



std::string get_current_time_string() {
	std::time_t time_now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

	std::string ret(30, '\0');
	std::strftime(&ret[0], ret.size(), "%Y-%m-%d %H:%M:%S", std::localtime(&time_now));

	return ret;
}

std::pair<std::string, std::string> get_input_output_file_paths() {

	std::string input_solr = "";
	std::string output_file_name = "";

	wxFileDialog OpenDialog(nullptr, "Choose a .solr file to load", wxEmptyString, wxEmptyString, _("Plan files (*.solr)|*.solr"), wxFD_OPEN, wxDefaultPosition);
	if (OpenDialog.ShowModal() == wxID_OK) // if the user clicked "Open" instead of cancel		
		input_solr = OpenDialog.GetPath().ToStdString();
		
	wxFileDialog OutputDialog(nullptr, "Choose a .txt file  where to output test result", wxEmptyString, wxEmptyString, _("Text files (*.txt)|*.txt"), wxFD_OPEN, wxDefaultPosition);
	if (OutputDialog.ShowModal() == wxID_OK) // if the user clicked "Open" instead of cancel		
		output_file_name = OutputDialog.GetPath().ToStdString();
	

	return std::make_pair(input_solr, output_file_name);
}

void TEST___hypothesis_1() {

	auto files = get_input_output_file_paths();
	std::string input_solr = files.first;
	std::string output_file_name = files.second;

	const std::vector<int> durations = { 2000, 2000, 2000 };
	const int time = 0;

	

	Simulation sim;		
	sim.load_plans(input_solr, durations);
	sim.chosen_detection_method = "Variable_Sampling_Detection";
	sim.move_to_time(time);

	auto map = sim.show_map();
	auto agents = sim.show_agents();
	
	std::vector <std::vector<plan_step>> expected_plans;
	for (size_t i = 0; i < agents.size(); i++)
		expected_plans.push_back(sim.get_expected_plan_from_time(time, agents[i]));


	std::unique_ptr<ICollision_Detection> cd = std::make_unique<Sampling_Detection>(map);



	auto start = std::chrono::high_resolution_clock::now();
	for (size_t i = 0; i < 1000; i++)
		cd->execute_detection(expected_plans, agents, time);
	auto end = std::chrono::high_resolution_clock::now();

	long long duration_variable = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();












	//So it cannot get benefir of being cached
	Simulation sim2;
	sim2.load_plans(input_solr, durations);
	sim2.chosen_detection_method = "Static_Sampling_Detection";
	sim2.move_to_time(time);

	auto map2 = sim2.show_map();
	auto agents2 = sim2.show_agents();

	std::vector <std::vector<plan_step>> expected_plans2;
	for (size_t i = 0; i < agents2.size(); i++)
		expected_plans2.push_back(sim2.get_expected_plan_from_time(time, agents2[i]));


	std::unique_ptr<ICollision_Detection> cd2 = std::make_unique<Static_Sampling_Detection>();

	auto start2 = std::chrono::high_resolution_clock::now();
	for (size_t i = 0; i < 1000; i++)
		cd2->execute_detection(expected_plans2, agents2, time);
	auto end2 = std::chrono::high_resolution_clock::now();

	long long duration_static = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2).count();





	std::ofstream myfile;
	myfile.open(output_file_name, std::ios_base::app);

	myfile << get_current_time_string() << std::endl;
	myfile << "duration_variable = " + std::to_string(duration_variable) << std::endl;
	myfile << "duration_static = " + std::to_string(duration_static) << std::endl << std::endl;

	myfile.close();
}




void TEST___hypothesis_4() {

	auto files = get_input_output_file_paths();
	std::string input_solr = files.first;
	std::string output_file_name = files.second;

	const std::vector<int> durations = { 2000, 2000, 2000 };
	const int time = 0;



	Simulation sim;
	sim.load_plans(input_solr, durations);
	sim.chosen_detection_method = "None";
	sim.move_to_time(time);

	auto map = sim.show_map();
	auto agents = sim.show_agents();

	std::vector <std::vector<plan_step>> expected_plans;
	for (size_t i = 0; i < agents.size(); i++)
		expected_plans.push_back(sim.get_expected_plan_from_time(time, agents[i]));

	std::vector<std::vector <std::vector<plan_step>>> vect_of_plans;
	for (size_t i = 0; i < expected_plans.size(); i++)
		vect_of_plans.push_back(expected_plans);


	std::vector<long long> res;

	std::unique_ptr<ICollision_Detection> cd;

	for (size_t i = 0; i < 4; i++) {

		auto act_plans = vect_of_plans[i];

		if (i == 0)
			cd = std::make_unique<Line_Detection>();
		else if (i == 1)
			cd = std::make_unique<Sampling_Detection>(map);
		else if (i == 2)
			cd = std::make_unique<Static_Sampling_Detection>();
		else if (i == 3)
			cd = std::make_unique<Rectangle_Detection>(2000);


		auto start = std::chrono::high_resolution_clock::now();
		for (size_t i = 0; i < 250; i++)
			cd->execute_detection(act_plans, agents, time);
		auto end = std::chrono::high_resolution_clock::now();




		//record result
		res.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

		//to avoid thermal throtling
		std::this_thread::sleep_for(std::chrono::seconds(5));
	}








	std::ofstream myfile;
	myfile.open(output_file_name, std::ios_base::app);

	myfile << get_current_time_string() << std::endl;
	myfile << "Line_Detection = " + std::to_string(res[0]) << std::endl;
	myfile << "Sampling_Detection = " + std::to_string(res[1]) << std::endl;
	myfile << "Static_Sampling_Detection = " + std::to_string(res[2]) << std::endl;
	myfile << "Rectangle_Detection = " + std::to_string(res[3]) << std::endl << std::endl;
	
	myfile.close();
}