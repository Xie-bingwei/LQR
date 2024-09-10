m = 1;   % Mass of ball
M = 5;   % Mass of car
L = 2;   % Length of pole
g = 10;  
d = 1;   % Friction coefficient
I = m * L^2 / 30;  % Moment of inertia
C = (M + m) * (I + m * L^2) - m^2 * L^2;  % Equation linearization

% First, analyze Kinematics of pendulum, we use Eulerian-Lagrangian method.
% Then, linearize the result matrix.
% Third, establish the State Space equations.
% Last, design LQR to control system.

A = [0 1 0 0; 
    0 -(I + m*L^2)*d/C (-m^2*L^2*g*(M + m)*(I + m*L^2))/C^2 0;
    0 0 0 1;
    0 m*L*d/C (M + m)*m*g*L/C 0];
B = [0; (I + m*L^2)/C; 0; -m*L/C];

eig(A);                                                                                                                             
rank(ctrb(A, B));

p = [-1, -2, -3, -4];
%p = [-4.2820, -1.9841, -1, -2];
K = place(A, B, p);

eig(A-B*K);

tspan = 0 : .05 : 10;
y0 = [0; 0; pi/6; .5];
[t, y] = ode45(@(t, y)pend_cart(y, I, m, M, L, g, d, -K*(y - [1; 0; 0; 0])), tspan, y0);
figure;

grid on;
hold on;                                                                                                                                                                                                                                            
plot(y(:, 1));
plot(y(:, 2));
plot(y(:, 3));
plot(y(:, 4));
legend({'x', '$$ \dot{x}$$', '$$\theta$$', '$$\dot{\theta}$$'}, 'interpreter', 'latex');
hold off;
figure;
for k = 1 : length(t)
    draw_pend(y(k, :), m, M, L);
end
