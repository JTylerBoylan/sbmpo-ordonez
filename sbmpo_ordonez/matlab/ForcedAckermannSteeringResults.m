%%

clc
clear
close all

stats = sbmpo_stats("../csv/stats.csv");
[path, nodes] = sbmpo_results("../csv/nodes.csv");

%% Nodes

node_count = nodes.buffer_size;
num_states = 5;
num_controls = 2;
state = reshape([nodes.nodes.state], [num_states node_count]);

x = state(1,:);
y = state(2,:);
q = state(3,:);
v = state(4,:);
g = state(5,:);

%% Path

path_count = path.path_size;
pstate = reshape([path.nodes.state], [num_states path_count]);

px = pstate(1,:);
py = pstate(2,:);
pq = pstate(3,:);
pv = pstate(4,:);
pg = pstate(5,:);

%% Plot

figure
hold on
grid on
plot(x,y,'ob') % all nodes
plot(px,py,'-g','LineWidth',3) % path
