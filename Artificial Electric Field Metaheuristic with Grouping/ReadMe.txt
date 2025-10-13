1. Data Files
Place your data files in the working directory

Supported format: .mat files containing:

CityNum: Number of cities

Demand: Task demands for each city

Distance: Distance matrix between locations

increase: Demand growth rate

ability: Robot capability

Travelcon: Travel constraints

v: Robot velocity

AntNum: Number of agents for path planning

2. Configuration
File Selection
Modify the fileNames array to include your data files:

matlab
fileNames = {'your_data_file1.mat', 'your_data_file2.mat'};
Experiment Settings
Maximum Iterations: Update Iter_save array

matlab
MaxIter  % Max iterations for each file
Experiment Repeats: Change the loop range

matlab
for cycle_num = 1:25  % Run 25 experiments per file
3. Key Parameters
Algorithm Parameters
matlab
ar = 7;               % Heuristic weight exponent
save_rate = 0.45;     % SR
best_rate = 0.45;     % SR
muta_rate = 0;        % Mutation rate(=0)
Taumax = inf;         % Upper bound for parameters
Taumin = 0.015;       % Lower bound for parameters(sigma in paper)
omega = 4;            % Threshold parameter
cF = 4;               % delta in paper
Robot Grouping
matlab
aggrenum = 2;         % Minimum number of groups
4. Running Experiments
Single File Testing:

matlab
for Init_k = 1      % Test only the 1rd file


matlab
for Init_k = 1:length(fileNames)
5. Output Results
Cycle Results: cyclebest contains best times for each experiment

Statistics: test_record stores mean and standard deviation

Iteration History: result_record tracks progress over iterations

6. Monitoring Progress
The algorithm displays iteration progress:

text
Iteration = 50, Min Time = 125.30
File Structure
Main algorithm file: multi_robot_task_allocation.m

Required functions: assignGrpsWithCDF.m, TextOutput_2.m
Note: The agent path output of AEFM-G is only partial because it outputs the common path of the group.
Data files: Various .mat files with problem instances