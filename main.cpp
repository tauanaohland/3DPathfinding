#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <cstdlib> 
#include "3k/AStar.h"
#include "LazyTheta/LazyTheta.h"
#include "LazyTheta/ThetaStar.h"
#include "jpsl/JPS.h"

float Euclidian(const Coord3D& a, const Coord3D& b)
{
	return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

float Voxel3D(const Coord3D& a, const Coord3D& b)
{
    float x_delta = std::abs(a.x - b.x);
    float y_delta = std::abs(a.y - b.y);
    float z_delta = std::abs(a.z - b.z);

    float d_min = std::min(std::min(x_delta, y_delta), z_delta);
    float d_max = std::max(std::max(x_delta, y_delta), z_delta);
    
    float d_sum = x_delta + y_delta + z_delta;
    float d_mid = d_sum - d_max - d_min;

    
    const float sqrt2 = std::sqrt(2);
    const float sqrt3 = std::sqrt(3);

    return (sqrt3 - sqrt2) * d_min + (sqrt2 - 1) * d_mid + d_max;
}

//traduzido direto pelo ChatGPT
void TestRandomScenarios(const std::vector<std::string>& scenario_paths, const std::vector<std::string>& expansion_policies, ISearcher* searcher, int scenarios_per_map_limit = 1000) {
    const std::string scenarios_map_path = "warframe";
    const std::string benchmarks_folder = "benchmarks";
    int scenarios_strike = 10000 / scenarios_per_map_limit;

    std::cout << "Iniciando testes sequenciais..." << std::endl;

    for (const auto& scenario_path : scenario_paths) 
    {

        for (const auto& expansion_policy : expansion_policies) 
        {

            auto start_at = std::chrono::high_resolution_clock::now();

            //random seed
            srand(123456);

            std::ifstream file(scenario_path);
            if (!file.is_open()) {
                std::cerr << "Failed to open file: " << scenario_path << std::endl;
                continue;
            }

            std::string line;
            std::getline(file, line); // Ignore the first line
            std::getline(file, line);
            std::string map_name = line;
            std::string map_file_path = scenarios_map_path + "/" + map_name;
            std::string benchmarks_file_path = benchmarks_folder + "/" + scenario_path.substr(scenario_path.find_last_of("/\\") + 1) + "." + searcher->GetName() + "." + expansion_policy + ".csv";

            std::fstream benchmark_file;
            std::ifstream check_file(benchmarks_file_path);
            if (check_file.is_open()) 
            {
                benchmark_file.open(benchmarks_file_path, std::ios::in | std::ios::out);
            }
            else 
            {
                benchmark_file.open(benchmarks_file_path, std::ios::out);
            }

            if (!benchmark_file.is_open()) 
            {
                std::cerr << "Failed to open benchmark file: " << benchmarks_file_path << std::endl;
                continue;
            }

            searcher->LoadMap(map_file_path);

            // Recover where it left off in case of loss.
            int count = 0;
            if (check_file.is_open()) 
            {
                count = std::count(std::istreambuf_iterator<char>(check_file), std::istreambuf_iterator<char>(), '\n');
            }

            if (count > 0)
            {
                count -= 1;
            }

            if (count > scenarios_per_map_limit) 
            {
                std::cout << "File " << benchmarks_file_path << " already completed or new file, skipping..." << std::endl;
                continue;
            }

            if (count == 0) 
            {
                benchmark_file << "Start, Goal, Cost, Total Time, Open Nodes, Path Nodes, Path Lenght\n";
            }

            std::vector<std::string> lines;
            while (std::getline(file, line)) {
                lines.push_back(line);
            }

            while (count < scenarios_per_map_limit) {
                // Choose a random scenario while respecting sequential order
                int offset = scenarios_strike * count;
                std::uniform_int_distribution<> dis(0, scenarios_strike - 1);
                int random_line = (int)rand() % scenarios_strike + offset;
                std::istringstream iss(lines[random_line]);
                std::vector<int> coords((std::istream_iterator<int>(iss)), std::istream_iterator<int>());

                // Get the start and goal coordinates
                JPSL::Point start(coords[0], coords[1], coords[2]);
                JPSL::Point goal(coords[3], coords[4], coords[5]);

                // Perform the pathfinding
                PathDescriptor descriptor = searcher->Search(start, goal, expansion_policy, 60);

                benchmark_file << "(" << start.x() << " " << start.y() << " " << start.z() << "),(" << goal.x() << " " << goal.y() << " " << goal.z() << "),"
                    << descriptor.cost << "," << descriptor.total_time << "," << descriptor.visited_nodes << "," << descriptor.path_nodes << "," << descriptor.path_lenght << "\n";
                count++;

                // Temporary saves.
                if (count % 10 == 0) {
                    std::cout << "Random scenarios, " << count << " of " << scenarios_per_map_limit << " using " << searcher->GetName() << " " << expansion_policy << " in map " << map_name << " done" << std::endl;
                    benchmark_file.flush();
                }
            }
            
            auto end_at = std::chrono::high_resolution_clock::now();
            double minutes = std::chrono::duration_cast<std::chrono::nanoseconds>(end_at - start_at).count() / 6E10;
            std::cout << "Time elapsed for the expansion policy \"" << expansion_policy << "\" in map " << map_name << ": " << minutes << " minutes (" << minutes / 60 << " hours)" << std::endl;
        }
    }
}

int IndividualTest()
{
	JPS jps(Voxel3D);
	//268 443 507 84 248 179
	Point start(268, 443, 507);
	Point goal(84, 248, 179);
	jps.LoadMap("warframe/BA1.3dmap");
	PathDescriptor path = jps.Search(start, goal, "k5-neighbors", 60);

	std::cout << "Path cost: " << path.cost << std::endl;
	std::cout << "Path length: " << path.path_lenght << std::endl;
	std::cout << "Path time: " << path.total_time << std::endl;
    std::cout << "Path Nodes:" << path.path_nodes << std::endl;
	std::cout << "Path visited nodes " << path.visited_nodes << std::endl;

    //system call to run "python plot_path"
    //system("python plot_path.py");

    return 0;
}

int IndividualThetaStarTest()
{
    ThetaStar searcher(Euclidian);  // Substitua 'searcher' por uma instância de 'ThetaStar'
    Point start(761, 306, 137);
    Point goal(537, 232, 129);
    searcher.LoadMap("warframe/A1.3dmap");

    // Realiza a busca com debug desativado
    PathDescriptor path = searcher.Search(start, goal, "k3-neighbors", 60);

    std::cout << "Path cost: " << path.cost << std::endl;
    std::cout << "Path length: " << path.path_lenght << std::endl;
    std::cout << "Path time: " << path.total_time << std::endl;
    std::cout << "Path Nodes:" << path.path_nodes << std::endl;
    std::cout << "Path visited nodes " << path.visited_nodes << std::endl;

    // Realiza a busca com debug ativado
    PathDescriptor descriptor = searcher.Search(start, goal, "k3-neighbors", 60, true);

    std::cout << "Path cost: " << descriptor.cost << std::endl;
    std::cout << "Path length: " << descriptor.path_lenght << std::endl;
    std::cout << "Path time: " << descriptor.total_time << std::endl;
    std::cout << "Path Nodes:" << descriptor.path_nodes << std::endl;
    std::cout << "Path visited nodes " << descriptor.visited_nodes << std::endl;

    // Chama o método PlotPath para plotar o caminho
    //descriptor.PlotPath("warframe/A1.3dmap");

    return 0;
}

int main()
{    
   
    //std::vector<std::string> scenarioPaths = { "warframe-3dscen/A1.3dmap.3dscen", "warframe-3dscen/BA1.3dmap.3dscen"  };
    //std::vector<std::string> scenarioPaths = {"warframe-3dscen/Complex.3dmap.3dscen", "warframe-3dscen/DA1.3dmap.3dscen"  };
    //std::vector<std::string> scenarioPaths = {"warframe-3dscen/EB1.3dmap.3dscen", "warframe-3dscen/FA1.3dmap.3dscen" };
   //std::vector<std::string> scenarioPaths = {"warframe-3dscen/Full1.3dmap.3dscen", "warframe-3dscen/DB1.3dmap.3dscen"  };
    std::vector<std::string> scenarioPaths = {"warframe-3dscen/BB1.3dmap.3dscen", "warframe-3dscen/EC1.3dmap.3dscen"  };
    //std::vector<std::string> scenarioPaths = { "warframe-3dscen/BB1.3dmap.3dscen" };
    std::vector<std::string> expancionPolicies = {"k4-neighbors"};
   
    
    //IndividualThetaStarTest();
    //TestRandomScenarios(scenarioPaths, expancionPolicies, new JPS(Voxel3D), 1000);
    //TestRandomScenarios(scenarioPaths, expancionPolicies, new Astar(Voxel3D), 1000);
    TestRandomScenarios(scenarioPaths, expancionPolicies, new ThetaStar(Voxel3D), 1000);
    //TestRandomScenarios(scenarioPaths, expancionPolicies, new Astar(Euclidian), 1000);
    //TestRandomScenarios(scenarioPaths, expancionPolicies, new ThetaStar(Euclidian), 1000);
    //TestRandomScenarios(scenarioPaths, expancionPolicies, new JPS(Euclidian), 1000);

    return 0;
}
