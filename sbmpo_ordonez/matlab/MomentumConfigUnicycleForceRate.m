%% Momentum Configuration Benchmark
% 2023-April-27
% Unicycle model with sampled longitudinal force and angular velocity

%state = [X Y th dX dY V]
% ATV weight = 500kg
% ATV max accel = 2m/s^2 -> max force = 1000N
% Izz = (500)*k^2 = 500*0.2^2 = 20 kgm^2

% Expected angular acceleration
% vmax = 10m/s
% rho_min = 2.74m % minimum turn radius
% over an 90 deg turn s = 2.74*pi/2 = 4.30 m
% delta_t = 4.30/5, close to 1
% ang_acc = pi/2 = 1.5 rad/sec^2
% max_torque = 20*1.5 = 30Nm

clear
close all

%% Vehicle parameters
mass = 500; % mass in kg
acc_max = 2; % max accel in m/s^2
vmax = 5; %m/s
rho_min = 2.74; % m
Fmax = mass*acc_max;
omega_max = vmax/rho_min;


%% Generate halton samples
n = 7; % number of samples
dim = 2; 

xmin = -Fmax; xmax = Fmax; % force
ymin = -omega_max; ymax = omega_max; % angular velocity

p = haltonset(dim);
p = net(p,n)


p(:,1) = xmin +(xmax-xmin)*p(:,1);
p(:,2) = ymin +(ymax-ymin)*p(:,2);

% overwrite samples
p = [-Fmax -omega_max;-Fmax 0;-Fmax omega_max; 0 0;Fmax -omega_max; Fmax 0; Fmax omega_max];


figure()
plot(p(:,1),p(:,2),'o','markerSize',10,'MarkerFaceColor','k');
grid on


%% Simulate vehicle motion with the provided samples
% this could be useful for debugging.

params.m = mass;
params.b = 0.0; % linear damping
params.fload = 0;

q0 = [0 0 0 0 0 0];
tspan = [0:0.01:0.2];

figure()

for iter = 1:n
        
    u = [p(iter,1) p(iter,2)];
    [t,q] = ode45(@(t,q)myUnicycleDynamicsForceRate(t,q,u,params),tspan,q0);

    hold on
    plot(q(:,1),q(:,2))
    xlabel('X(m)')
    ylabel('Y(m)')
 
    axis equal
    grid on
end
figure()
subplot(2,1,1)
plot(t,q(:,3))
subplot(2,1,2)
plot(t,q(:,5))

%% Parameters

patch_start = -1;
min_vel_required = 1.53;

runs = 1;

params = struct;
params.max_iterations = 200000;

params.max_generations = 100;

params.horizon_time = 0.5;
params.num_states = 4;
params.num_controls = 2;
params.grid_resolution = [0.025; 0.025; 0.025; -1];

params.start_state = [0; 0; 0; 0];
params.goal_state =  [15; 15; 0; 0]; 

%params.goal_state =  [patch_start; min_vel_required; 0]; 


params.branchout_factor = n;
params.branchouts = p';
% params.branchouts = [
%     [p(1,1); p(1,2)], ...
%     [p(2,1); p(2,2)], ...
%     [p(3,1); p(3,2)], ...
%     [p(4,1); p(4,2)], ...
%     [p(5,1); p(5,2)], ...
%     [p(6,1); p(6,2)], ...
%     [p(7,1); p(7,2)], ...
%     [p(8,1); p(8,2)], ...
%     [p(9,1); p(9,2)]
%     ];
%% Write config file

sbmpo_config("../csv/config.csv", params, runs);
