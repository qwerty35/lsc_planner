% Set start and goal points
% Example: circle swap with 8 agents
N = 8; % The number of agents
circle_radius = 4;
z_2d = 1;

start_points = zeros(N,3);
goal_points = zeros(N,3);
for qi = 1:N
    theta = 2 * pi * (qi-1) / N;
    start_points(qi,:) = [circle_radius * cos(theta), circle_radius * sin(theta), z_2d];
    goal_points(qi,:) = [-circle_radius * cos(theta), -circle_radius * sin(theta), z_2d];
end

% Parameters (Must be string)
file_name = 'mission.json'; % must end with '.json'
max_vel = '[1.0, 1.0, 1.0]';
max_acc = '[2.0, 2.0, 2.0]';
radius = '0.15';
downwash = '2.0';
dimension = '[-5.0, -5.0, 0.0, 5.0, 5.0, 2.5]';

% String for mission 
agents = '  "agents": [\n';
type = '    {"type": ';
crazyflie = '"crazyflie", ';
cid = '"cid": ';
start = ', "start": [';
goal = '], "goal": [';
next = ']},\n';
eol = ']}\n  ],\n';
intro1 = '{\n"quadrotors": {\n"crazyflie": {"max_vel":';
intro2 = ',\n"max_acc": ';
intro3 = ',\n"radius": ';
intro4 = ',\n"nominal_velocity": 1.0,\n"downwash": ';
intro5 = '},\n"default": {"max_vel": [1.0, 1.0, 1.0],\n"max_acc": [2.0, 2.0, 1.0],\n"radius": 0.15,\n"nominal_velocity": 1.0,\n"downwash": 2.0}\n},\n\n"world": [\n{"dimension": ';
intro6 = ', }\n],\n';
intro = [intro1, max_vel, intro2, max_acc, intro3, radius, intro4, downwash, intro5, dimension, intro6];
outro = '\n"obstacles": [\n]\n}';

% Generate a random set of initial and final positions
fileID = fopen(file_name,'w');
fprintf(fileID,intro);
mission = agents;
for qi = 1:N
    start_point = [num2str(start_points(qi,1)), ', ', ...
                   num2str(start_points(qi,2)), ', ', ...
                   num2str(start_points(qi,3))];
    goal_point = [num2str(goal_points(qi,1)), ', ', ...
                  num2str(goal_points(qi,2)), ', ', ...
                  num2str(goal_points(qi,3))];
                 
    mission = [mission, type, crazyflie, cid, num2str(qi), start, start_point, goal, goal_point];
    if(qi < N)
        mission = [mission, next];
    else
        mission = [mission, eol];
    end
end
fprintf(fileID,mission);
fprintf(fileID,outro);

fclose(fileID);
