% Momentum


close all

stats = sbmpo_stats("../csv/stats.csv");
[path, nodes] = sbmpo_results("../csv/nodes.csv");
%% Path

t = zeros(1, path.path_size);
x = zeros(1, path.path_size);
y = zeros(1, path.path_size);
th = zeros(1,path.path_size);
v = zeros(1, path.path_size);
u = zeros(2, path.path_size-1);

for nd = 1:path.path_size
    t(nd) = path.nodes(nd).g;
    x(nd) = path.nodes(nd).state(1);
    y(nd) = path.nodes(nd).state(2);
    th(nd) = path.nodes(nd).state(3);
    v(nd) = path.nodes(nd).state(4);

        

    if (nd ~= 1)
        u(nd,1) = path.nodes(nd).control(1);
        u(nd,2) = path.nodes(nd).control(2);
    else
        u(nd,1) = NaN;
        u(nd,2) = NaN;
    end
end

figure(1)

% Plot X vs T
subplot(2,1,1)
    plot(x,y)
    title("Position")
    xlabel("x (m)")
    ylabel("y (m)")
    axis equal


 subplot(2,1,2)
    plot(t,v)
    xlabel("Time (s)")
    ylabel("V (m/s)")



% Plot U vs T
%subplot(2,2,4)
%plot(t,u)
%title("Control")
%xlabel("Time (s)")
%ylabel("U (m/s^2)")

%% State space

x_all = zeros(1, nodes.buffer_size);
y_all = zeros(1, nodes.buffer_size);
for nd = 1:nodes.buffer_size
    x_all(nd) = nodes.nodes(nd).state(1);
    y_all(nd) = nodes.nodes(nd).state(2);
end

% Plot V vs X
figure(2)
    plot(x_all, y_all, 'ob','MarkerSize',5);
    hold on
    plot(x,y,'-g','LineWidth',5)
    xlabel("X");
    ylabel("y");
keyboard()
%axis([-5 5 -5 5])
% draw velocity obstacles
patch_start = -1;
patch_end = 5;
xs = patch_start:0.001:patch_end;
mu_patch = 0.1;
load_to_traction = 1.2;
g = 9.8;
a =mu_patch*g - load_to_traction*mu_patch*g; % Expected deceleration through patch
vs = sqrt(a*2*(xs-patch_end))
hold on
plot(xs,vs,'-k','LineWidth',4)
axis equal

keyboard()
%% Time heuristic

v_opt = linspace(-15, 15, 100);
x_opt = -v_opt.*abs(v_opt)./(2*max(u));

plot(x_opt, v_opt, '--k')