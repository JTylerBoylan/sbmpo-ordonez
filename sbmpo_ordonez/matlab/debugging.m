%%

clc
clear
close all

stats = sbmpo_stats("../csv/stats.csv");
[path, nodes] = sbmpo_results("../csv/nodes.csv");

%%


% random sample
samp_prob = 0.01;
rsamp = ~logical(randi(1/samp_prob,1, nodes.buffer_size)-1);
node_samp = nodes.nodes(rsamp);
samp_size = numel(node_samp);

% extract data
num_states = 4;
state = reshape([node_samp.state], [num_states samp_size]);
g = [nodes.nodes.g];
f = [nodes.nodes.f];
h = f - g;

% colors
g_col = (g - min(g)) / (max(g) - min(g));
f_col = (f - min(f)) / (max(f) - min(f));
h_col = (h - min(h)) / (max(h) - min(h));

% plot
figure
hold on
for nd = 1:samp_size
   nx = state(1,nd);
   ny = state(2,nd);
   nq = state(3,nd);
   nv = state(4,nd);
   nhc = h_col(nd);
   nfc = f_col(nd);
   ngc = g_col(nd);
   plot(nx,ny,'x','Color',[ngc, 1-ngc, 0])
end