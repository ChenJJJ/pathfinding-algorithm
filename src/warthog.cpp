// warthog.cpp
//
// @author: dharabor
// @created: August 2012
//

#include "cfg.h"
#include "flexible_astar.h"
#include "gridmap.h"
#include "gridmap_expansion_policy.h"
#include "jps_expansion_policy.h"
#include "jps_expansion_policy_wgm.h"
#include "jps2_expansion_policy.h"
#include "jpsplus_expansion_policy.h"
#include "jps2plus_expansion_policy.h"
#include "octile_heuristic.h"
#include "manhattan_heuristic.h"
#include "scenario_manager.h"
#include "weighted_gridmap.h"
#include "wgridmap_expansion_policy.h"
#include "zero_heuristic.h"
#include "cpd_heuristic.h"

#include "getopt.h"

#include <iomanip>
#include <sstream>
#include <unordered_map>
#include <memory>

#include <stdio.h>
#include <stdint.h>
#include <numeric>
#include <algorithm>
#include "ScenarioLoader.h"
//#include "Timer_cpd.h"
#include "Entry.h"
#include <vector>

#include "timer.h"

using namespace std;
// check computed solutions are optimal
int checkopt = 0;
// print debugging info during search
int verbose = 0;
// display program help on startup
int print_help = 0;
// treat the map as a weighted-cost grid
int wgm = 0;

int current_id=0;

void
help()
{
	std::cerr << "valid parameters:\n"
	<< "--alg [astar | jps | jps2 | jps+ | jps2+ | jps | sssp ]\n"
	<< "--scen [scenario filename]\n"
	<< "--gen [map filename]\n"
	<< "--wgm (optional)\n"
	<< "--checkopt (optional)\n"
	<< "--verbose (optional)\n";
}

void
check_optimality(double len, warthog::experiment* exp)
{
	if(!checkopt)
	{
		return;
	}

	uint32_t precision = 1;
	int epsilon = (warthog::ONE / (int)pow(10, precision)) / 2;

	int32_t int_len = len * warthog::ONE;
	int32_t int_opt = exp->distance() * warthog::ONE;

	for(int i = 10; i <= pow(10, precision); i = i*10)
	{
		int last_digit = int_len % i;
		if(last_digit >= (i/2))
		{
			int_len += (i - last_digit);
		}
	}

	int32_t delta = abs(int_len - int_opt);
	if( abs(delta - epsilon) > epsilon)
	{
		std::stringstream strpathlen;
		strpathlen << std::fixed << std::setprecision(exp->precision());
		strpathlen << len*warthog::ONE;

		std::stringstream stroptlen;
		stroptlen << std::fixed << std::setprecision(exp->precision());
		stroptlen << exp->distance() * warthog::ONE;

		std::cerr << std::setprecision(exp->precision());
		std::cerr << "optimality check failed!" << std::endl;
		std::cerr << std::endl;
		std::cerr << "optimal path length: "<<stroptlen.str()
			<<" computed length: ";
		std::cerr << strpathlen.str()<<std::endl;
		std::cerr << "precision: " << precision << " epsilon: "<<epsilon<<std::endl;
		std::cerr<< "delta: "<< delta << std::endl;
		exit(1);
	}
}

void
run_jpsplus(warthog::scenario_manager& scenmgr)
{
    warthog::gridmap map(scenmgr.get_experiment(0)->map().c_str());
	warthog::jpsplus_expansion_policy expander(&map);
	warthog::octile_heuristic heuristic(map.width(), map.height());

	warthog::flexible_astar<
		warthog::octile_heuristic,
	   	warthog::jpsplus_expansion_policy> astar(&heuristic, &expander);
	astar.set_verbose(verbose);

	std::cout << "id\talg\texpd\tgend\ttouched\ttime\tcost\tsfile\n";
	for(unsigned int i=0; i < scenmgr.num_experiments(); i++)
	{
		warthog::experiment* exp = scenmgr.get_experiment(i);

		int startid = exp->starty() * exp->mapwidth() + exp->startx();
		int goalid = exp->goaly() * exp->mapwidth() + exp->goalx();
		double len = astar.get_length(
				map.to_padded_id(startid),
			   	map.to_padded_id(goalid));
		if(len == warthog::INF)
		{
			len = 0;
		}

		std::cout << i<<"\t" << "jps+" << "\t" 
		<< astar.get_nodes_expanded() << "\t" 
		<< astar.get_nodes_generated() << "\t"
		<< astar.get_nodes_touched() << "\t"
		<< astar.get_search_time()  << "\t"
		<< len << "\t" 
		<< scenmgr.last_file_loaded() << std::endl;

		check_optimality(len, exp);
	}
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_jps2plus(warthog::scenario_manager& scenmgr)
{
    warthog::gridmap map(scenmgr.get_experiment(0)->map().c_str());
	warthog::jps2plus_expansion_policy expander(&map);
	warthog::octile_heuristic heuristic(map.width(), map.height());

	warthog::flexible_astar<
		warthog::octile_heuristic,
	   	warthog::jps2plus_expansion_policy> astar(&heuristic, &expander);
	astar.set_verbose(verbose);

	std::cout << "id\talg\texpd\tgend\ttouched\ttime\tcost\tsfile\n";
	for(unsigned int i=0; i < scenmgr.num_experiments(); i++)
	{
		warthog::experiment* exp = scenmgr.get_experiment(i);

		int startid = exp->starty() * exp->mapwidth() + exp->startx();
		int goalid = exp->goaly() * exp->mapwidth() + exp->goalx();
		double len = astar.get_length(
				map.to_padded_id(startid),
			   	map.to_padded_id(goalid));
		if(len == warthog::INF)
		{
			len = 0;
		}

		std::cout << i<<"\t" << "jps2+" << "\t" 
		<< astar.get_nodes_expanded() << "\t" 
		<< astar.get_nodes_generated() << "\t"
		<< astar.get_nodes_touched() << "\t"
		<< astar.get_search_time()  << "\t"
		<< len << "\t" 
		<< scenmgr.last_file_loaded() << std::endl;

		check_optimality(len, exp);
	}
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_jps2(warthog::scenario_manager& scenmgr)
{
    warthog::gridmap map(scenmgr.get_experiment(0)->map().c_str());
	warthog::jps2_expansion_policy expander(&map);
	warthog::octile_heuristic heuristic(map.width(), map.height());

	warthog::flexible_astar<
		warthog::octile_heuristic,
	   	warthog::jps2_expansion_policy> astar(&heuristic, &expander);
	astar.set_verbose(verbose);

	std::cout << "id\talg\texpd\tgend\ttouched\ttime\tcost\tsfile\n";
	for(unsigned int i=0; i < scenmgr.num_experiments(); i++)
	{
		warthog::experiment* exp = scenmgr.get_experiment(i);

		int startid = exp->starty() * exp->mapwidth() + exp->startx();
		int goalid = exp->goaly() * exp->mapwidth() + exp->goalx();
		double len = astar.get_length(
				map.to_padded_id(startid),
			   	map.to_padded_id(goalid));
		if(len == warthog::INF)
		{
			len = 0;
		}

		std::cout << i<<"\t" << "jps2" << "\t" 
		<< astar.get_nodes_expanded() << "\t" 
		<< astar.get_nodes_generated() << "\t"
		<< astar.get_nodes_touched() << "\t"
		<< astar.get_search_time()  << "\t"
		<< len << "\t" 
		<< scenmgr.last_file_loaded() << std::endl;

		check_optimality(len, exp);
	}
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_jps(warthog::scenario_manager& scenmgr)
{
    warthog::gridmap map(scenmgr.get_experiment(0)->map().c_str());
	warthog::jps_expansion_policy expander(&map);
	warthog::octile_heuristic heuristic(map.width(), map.height());

	warthog::flexible_astar<
		warthog::octile_heuristic,
	   	warthog::jps_expansion_policy> astar(&heuristic, &expander);
	astar.set_verbose(verbose);

	std::cout << "id\talg\texpd\tgend\ttouched\ttime\tcost\tsfile\n";
	for(unsigned int i=0; i < scenmgr.num_experiments(); i++)
	{
		warthog::experiment* exp = scenmgr.get_experiment(i);

		int startid = exp->starty() * exp->mapwidth() + exp->startx();
		int goalid = exp->goaly() * exp->mapwidth() + exp->goalx();
		double len = astar.get_length(
				map.to_padded_id(startid),
			   	map.to_padded_id(goalid));
		if(len == warthog::INF)
		{
			len = 0;
		}

		std::cout << i<<"\t" << "jps" << "\t" 
		<< astar.get_nodes_expanded() << "\t" 
		<< astar.get_nodes_generated() << "\t"
		<< astar.get_nodes_touched() << "\t"
		<< astar.get_search_time()  << "\t"
		<< len << "\t" 
		<< scenmgr.last_file_loaded() << std::endl;

		check_optimality(len, exp);
	}
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

//astar with octile heuristic function
void
run_astar(warthog::scenario_manager& scenmgr)
{
    warthog::gridmap map(scenmgr.get_experiment(0)->map().c_str());
	warthog::gridmap_expansion_policy expander(&map);
	warthog::octile_heuristic heuristic(map.width(), map.height());

warthog::flexible_astar<
	warthog::octile_heuristic,
   	warthog::gridmap_expansion_policy> astar(&heuristic, &expander);
astar.set_verbose(verbose);

	std::cout << "id\talg\texpd\tgend\ttouched\ttime\tcost\tsfile\n";
	for(unsigned int i=0; i < scenmgr.num_experiments(); i++)
	{
		warthog::experiment* exp = scenmgr.get_experiment(i);

		int startid = exp->starty() * exp->mapwidth() + exp->startx();
		int goalid = exp->goaly() * exp->mapwidth() + exp->goalx();
		double len = astar.get_length(
				map.to_padded_id(startid), 
				map.to_padded_id(goalid));
		if(len == warthog::INF)                                                                   
		{
			len = 0;
		}

		std::cout << i<<"\t" << "astar" << "\t" 
		<< astar.get_nodes_expanded() << "\t" 
		<< astar.get_nodes_generated() << "\t"
		<< astar.get_nodes_touched() << "\t"
		<< astar.get_search_time()  << "\t"
		<< len << "\t" 
		<< scenmgr.last_file_loaded() << std::endl;

		check_optimality(len, exp);
	}
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_astar_cpd(warthog::scenario_manager& scenmgr, void *reference)
{

	warthog::gridmap map(scenmgr.get_experiment(0)->map().c_str());
	warthog::gridmap_expansion_policy expander(&map);
	warthog::cpd_heuristic heuristic(map.width(), map.height(), map.header_width(), map.header_height());

	warthog::flexible_astar<
		warthog::cpd_heuristic,
	   	warthog::gridmap_expansion_policy> astar(&heuristic, &expander);
	astar.set_verbose(verbose);
	//astar.set_hscale('.');  
	
	std::vector<double> h_array;
	std::vector<int> exp_id;
	
	h_array.resize(0);
	exp_id.resize(0);
	for (int i=0; i<= map.header_height() *map.header_width() + map.header_width(); i++)
		{
			h_array.push_back(0);
			exp_id.push_back(0);
		}

	int bomb_cost=55;
	unsigned int seed = 611;
	unsigned int radius = 1; 
	map.bomb(seed,radius,bomb_cost);

	std::cout << "id\talg\texpd\tgend\ttouched\ttime\tcost\tsfile\n";
	for(unsigned int i=0; i < scenmgr.num_experiments(); i++)
	{
		warthog::experiment* exp = scenmgr.get_experiment(i);

		int startid = exp->starty() * exp->mapwidth() + exp->startx();
		int goalid = exp->goaly() * exp->mapwidth() + exp->goalx();

		bomb_cost -=5;			//bomb cost will vary between different experiment, normally decay 
		if(bomb_cost<=1){
			bomb_cost=1;
		}
		map.bomb(seed, radius, bomb_cost);

		double len = astar.get_length_cpd(
				map.to_padded_id(startid), 
				map.to_padded_id(goalid), reference, h_array, exp_id, current_id);
		if(len == warthog::INF)                                                                   
		{
			len = 0;
		}

		std::cout << i<<"\t" << "astar" << "\t" 
		<< astar.get_nodes_expanded() << "\t" 
		<< astar.get_nodes_generated() << "\t"
		<< astar.get_nodes_touched() << "\t"
		<< astar.get_search_time()  << "\t"
		<< len << "\t" 
		<< scenmgr.last_file_loaded() << std::endl;

		check_optimality(len, exp);
		current_id++;
	}
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_wgm_astar(warthog::scenario_manager& scenmgr)
{
    warthog::weighted_gridmap map(scenmgr.get_experiment(0)->map().c_str());
	warthog::wgridmap_expansion_policy expander(&map);
	warthog::octile_heuristic heuristic(map.width(), map.height());

	warthog::flexible_astar<
		warthog::octile_heuristic,
	   	warthog::wgridmap_expansion_policy> astar(&heuristic, &expander);
	astar.set_verbose(verbose);
    // cheapest terrain (movingai benchmarks) has ascii value '.'; we scale
    // all heuristic values accordingly (otherwise the heuristic doesn't 
    // impact f-values much and search starts to behave like dijkstra)
    astar.set_hscale('.');  
	
	// unsigned int radius = 1;
	// unsigned int seed =510;
	// unsigned int bomb_cost = 100;
	// map.bomb(seed, radius, bomb_cost);

	std::cout << "id\talg\texpd\tgend\ttouched\ttime\tcost\tsfile\n";
	for(unsigned int i=0; i < scenmgr.num_experiments(); i++)
	{
		warthog::experiment* exp = scenmgr.get_experiment(i);

		int startid = exp->starty() * exp->mapwidth() + exp->startx();
		int goalid = exp->goaly() * exp->mapwidth() + exp->goalx();
		
		//modified part
		// bomb_cost -=1;
		// map.bomb(seed, radius, bomb_cost);

		double len = astar.get_length(
				map.to_padded_id(startid), 
				map.to_padded_id(goalid));
		if(len == warthog::INF)
		{
			len = 0;
		}

		std::cout << i<<"\t" << "astar_wgm" << "\t" 
		<< astar.get_nodes_expanded() << "\t" 
		<< astar.get_nodes_generated() << "\t"
		<< astar.get_nodes_touched() << "\t"
		<< astar.get_search_time()  << "\t"
		<< len << "\t" 
		<< scenmgr.last_file_loaded() << std::endl;

		check_optimality(len, exp);
	}
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_wgm_sssp(warthog::scenario_manager& scenmgr)
{
    warthog::weighted_gridmap map(scenmgr.get_experiment(0)->map().c_str());
	warthog::wgridmap_expansion_policy expander(&map);
	warthog::zero_heuristic heuristic(map.width(), map.height());

	warthog::flexible_astar<
		warthog::zero_heuristic,
	   	warthog::wgridmap_expansion_policy> astar(&heuristic, &expander);
	astar.set_verbose(verbose);

	std::cout << "id\talg\texpd\tgend\ttouched\ttime\tsfile\n";
	for(unsigned int i=0; i < scenmgr.num_experiments(); i++)
	{
		warthog::experiment* exp = scenmgr.get_experiment(i);

		int startid = exp->starty() * exp->mapwidth() + exp->startx();
		astar.get_length(map.to_padded_id(startid), warthog::INF);

		std::cout << i<<"\t" << "sssp_wgm" << "\t" 
		<< astar.get_nodes_expanded() << "\t" 
		<< astar.get_nodes_generated() << "\t"
		<< astar.get_nodes_touched() << "\t"
		<< astar.get_search_time()  << "\t"
		<< scenmgr.last_file_loaded() << std::endl;
	}
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}

void
run_jps_wgm(warthog::scenario_manager& scenmgr)
{
    warthog::weighted_gridmap map(scenmgr.get_experiment(0)->map().c_str());
	warthog::jps_expansion_policy_wgm expander(&map);
	warthog::octile_heuristic heuristic(map.width(), map.height());

	warthog::flexible_astar<
		warthog::octile_heuristic,
	   	warthog::jps_expansion_policy_wgm> astar(&heuristic, &expander);
	astar.set_verbose(verbose);
    // cheapest terrain (movingai benchmarks) has ascii value '.'; we scale
    // all heuristic values accordingly (otherwise the heuristic doesn't 
    // impact f-values much and search starts to behave like dijkstra)
    astar.set_hscale('.');  

	std::cout << "id\talg\texpd\tgend\ttouched\ttime\tcost\tsfile\n";
	for(unsigned int i=0; i < scenmgr.num_experiments(); i++)
	{
		warthog::experiment* exp = scenmgr.get_experiment(i);

		int startid = exp->starty() * exp->mapwidth() + exp->startx();
		int goalid = exp->goaly() * exp->mapwidth() + exp->goalx();
		double len = astar.get_length(
				map.to_padded_id(startid),
			   	map.to_padded_id(goalid));
		if(len == warthog::INF)
		{
			len = 0;
		}

		std::cout << i<<"\t" << "jps_wgm" << "\t" 
		<< astar.get_nodes_expanded() << "\t" 
		<< astar.get_nodes_generated() << "\t"
		<< astar.get_nodes_touched() << "\t"
		<< astar.get_search_time()  << "\t"
		<< len << "\t" 
		<< scenmgr.last_file_loaded() << std::endl;

		check_optimality(len, exp);
	}
	std::cerr << "done. total memory: "<< astar.mem() + scenmgr.mem() << "\n";
}


void LoadMap(const char *fname, std::vector<bool> &map, int &w, int &h);

struct stats {
	std::vector<double> times;
	std::vector<xyLoc> path;
	std::vector<int> lengths;

	double GetTotalTime()
	{
		return std::accumulate(times.begin(), times.end(), 0.0);
	}
	double GetMaxTimestep()
	{
		return *std::max_element(times.begin(), times.end());
	}
	double Get20MoveTime()
	{
		for (unsigned int x = 0; x < lengths.size(); x++)
			if (lengths[x] >= 20)
				return std::accumulate(times.begin(), times.begin()+1+x, 0.0);
		return GetTotalTime();
	}
	double GetPathLength()
	{
		double len = 0;
		for (int x = 0; x < (int)path.size()-1; x++)
		{
			if (path[x].x == path[x+1].x || path[x].y == path[x+1].y)
			{
				len++;
			}
			else {
				len += 1.4142;
			}
		}
		return len;
	}
	bool ValidatePath(int width, int height, const std::vector<bool> &mapData)
	{
		for (int x = 0; x < (int)path.size()-1; x++)
		{
			if (abs(path[x].x - path[x+1].x) > 1)
				return false;
			if (abs(path[x].y - path[x+1].y) > 1)
				return false;
			if (!mapData[path[x].y*width+path[x].x])
				return false;
			if (!mapData[path[x+1].y*width+path[x+1].x])
				return false;
			if (path[x].x != path[x+1].x && path[x].y != path[x+1].y)
			{
				if (!mapData[path[x+1].y*width+path[x].x])
					return false;
				if (!mapData[path[x].y*width+path[x+1].x])
					return false;
			}
		}
		return true;
	}
};

void LoadMap(const char *fname, std::vector<bool> &map, int &width, int &height)
	{
		FILE *f;
		f = fopen(fname, "r");
		if (f)
    	{
			fscanf(f, "type octile\nheight %d\nwidth %d\nmap\n", &height, &width);
			map.resize(height*width);
			for (int y = 0; y < height; y++)
			{
				for (int x = 0; x < width; x++)
				{
					char c;
					do {
						fscanf(f, "%c", &c);
					} while (isspace(c));
					map[y*width+x] = (c == '.' || c == 'G' || c == 'S');
					//printf("%c", c);
				}
				//printf("\n");
			}
			fclose(f);
    	}
	}

int 
main(int argc, char** argv)
{
	// parse arguments
	warthog::util::param valid_args[] = 
	{
		{"scen",  required_argument, 0, 0},
		{"alg",  required_argument, 0, 1},
		{"gen", required_argument, 0, 3},
		{"help", no_argument, &print_help, 1},
		{"checkopt",  no_argument, &checkopt, 1},
		{"verbose",  no_argument, &verbose, 1},
		{"wgm",  no_argument, &wgm, 1}
	};

	warthog::util::cfg cfg;
	cfg.parse_args(argc, argv, valid_args);

    if(print_help)
    {
		help();
        exit(0);
    }

	std::string sfile = cfg.get_param_value("scen");
	std::string alg = cfg.get_param_value("alg");
	std::string gen = cfg.get_param_value("gen");

    // generate scenarios
	if(gen != "")
	{
		warthog::scenario_manager sm;
		warthog::gridmap gm(gen.c_str());
		sm.generate_experiments(&gm, 1000) ;
		sm.write_scenario(std::cout);
        exit(0);
	}

	// run experiments
	if(alg == "" || sfile == "")
	{
        std::cerr << "Err. Must specify a scenario file and search algorithm. Try --help for options.\n";
		exit(0);
	}

	warthog::scenario_manager scenmgr;
	scenmgr.load_scenario(sfile.c_str());
    std::cerr << "wgm: " << (wgm ? "true" : "false") << std::endl;

	if(alg == "jps+")
	{
		run_jpsplus(scenmgr);
	}

	if(alg == "jps2")
	{
		run_jps2(scenmgr);
	}

	if(alg == "jps2+")
	{
		run_jps2plus(scenmgr);
	}

    if(alg == "jps")
    {
        if(wgm)
        {
            run_jps_wgm(scenmgr);
        }
        else
        {
            run_jps(scenmgr);
        }
    }

	if(alg == "astar_cpd")
	{
	
	char filename[255];
	std::vector<xyLoc> thePath;
	std::vector<bool> mapData;
	int width=0, height=0;

	LoadMap(argv[5], mapData, width, height);
	sprintf(filename, "%s-%s", warthog::GetName(), argv[5]);
	warthog::PreprocessMap(mapData, width, height, filename);

	void *reference = warthog::PrepareForSearch(mapData, width, height, filename);

        if(wgm) 
        { 
            run_wgm_astar(scenmgr); 
        }
        else 
        { 
            run_astar_cpd(scenmgr, reference); 
        }
	}

	if(alg == "astar")
	{

        if(wgm) 
        { 
            run_wgm_astar(scenmgr); 
        }
        else 
        { 
            run_astar(scenmgr); 
        }
	}

	if(alg == "sssp")
	{
        if(wgm) 
        { 
            run_wgm_sssp(scenmgr); 
        }
        else 
        { 
            //run_astar(scenmgr); 
        }
	}

	if(alg == "cpd")
	{
	
	char filename[255];
	std::vector<xyLoc> thePath;
	std::vector<bool> mapData;
	int width, height;
	// bool pre = false;
	// bool run = false;

	// if (argc != 4)
	// {
	// 	printf("Invalid Arguments\nUsage %s <flag> <map> <scenario>\n", argv[0]);
	// 	printf("Flags:\n");
	// 	printf("\t-full : Preprocess map and run scenario\n");
	// 	printf("\t-pre : Preprocess map\n");
	// 	printf("\t-run : Run scenario without preprocessing\n");
	// 	exit(0);
	// }
	// if (strcmp(argv[1], "-full") == 0)
	// {
	// 	pre = run = true;
	// }
	// else if (strcmp(argv[1], "-pre") == 0)
	// {
	// 	pre = true;
	// }
	// else if (strcmp(argv[1], "-run") == 0)
	// {
	// 	run = true;
	// }
	// else {
    //     printf("Invalid Arguments\nUsage %s <flag> <map> <scenario>\n", argv[0]);
	// 	printf("Flags:\n");
    //     printf("\t-full : Preprocess map and run scenario\n");
    //     printf("\t-pre : Preprocess map\n");
    //     printf("\t-run : Run scenario without preprocessing\n");
    //     exit(0);
	// }
	
	LoadMap(argv[5], mapData, width, height);
	sprintf(filename, "%s-%s", warthog::GetName(), argv[5]);

	// if (pre)
	// {
		warthog::weighted_gridmap map(scenmgr.get_experiment(0)->map().c_str());

		warthog::PreprocessMap(mapData, width, height, filename);
	// }
	
	// if (!run)
	// {
	// 	return 0;
	// }

	void *reference = warthog::PrepareForSearch(mapData, width, height, filename);

	ScenarioLoader scen(argv[4]);

	//Timer_cpd t;
	warthog::timer t;

	std::vector<stats> experimentStats;
	
	for (int x = 0; x < scen.GetNumExperiments(); x++)
    {
		//printf("%d of %d\n", x+1, scen.GetNumExperiments());
		thePath.resize(0);
		experimentStats.resize(x+1);
		bool done;
		do {
			xyLoc s, g;
			s.x = scen.GetNthExperiment(x).GetStartX();
			s.y = scen.GetNthExperiment(x).GetStartY();
			g.x = scen.GetNthExperiment(x).GetGoalX();
			g.y = scen.GetNthExperiment(x).GetGoalY();

			// t.StartTimer_cpd();
			t.warthog::timer::start();
			done = warthog::GetPath(reference, s, g, thePath);
			// t.EndTimer_cpd();
			t.warthog::timer::stop();

			// experimentStats[x].times.push_back(t.GetElapsedTime());
			experimentStats[x].times.push_back((t.warthog::timer::elapsed_time_nano()/1000000000));

			experimentStats[x].lengths.push_back(thePath.size());
			for (unsigned int t = experimentStats[x].path.size(); t < thePath.size(); t++)
				experimentStats[x].path.push_back(thePath[t]);
		} while (done == false);

    }
	
	//printf("%d\t",experimentStats.size());
		for (unsigned int x = 0; x < experimentStats.size(); x++)
		{
			//printf("%d\n",x);
			printf("%s\ttotal-time\t%f\tmax-time-step\t%f\ttime-20-moves\t%f\ttotal-len\t%f\tsubopt\t%f\t", argv[4],
				experimentStats[x].GetTotalTime(), experimentStats[x].GetMaxTimestep(), experimentStats[x].Get20MoveTime(),
				experimentStats[x].GetPathLength(), 
				experimentStats[x].GetPathLength() == scen.GetNthExperiment(x).GetDistance() ? 1.0 : 
				experimentStats[x].GetPathLength() / scen.GetNthExperiment(x).GetDistance()
			);
			
			if (experimentStats[x].path.size() == 0 ||
				(experimentStats[x].ValidatePath(width, height, mapData) &&
				scen.GetNthExperiment(x).GetStartX() == experimentStats[x].path[0].x &&
				scen.GetNthExperiment(x).GetStartY() == experimentStats[x].path[0].y &&
				scen.GetNthExperiment(x).GetGoalX() == experimentStats[x].path.back().x &&
				scen.GetNthExperiment(x).GetGoalY() == experimentStats[x].path.back().y))
			{
				printf("valid\n");
			}
			else {
				printf("invalid\n");
			}
			//printf("%d\t",experimentStats.size());
		}
		return 0;

	}

	
}

