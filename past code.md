Q1(b)LQR controller
clear;
% Define the A matrix
A = [0, 0, 1, 0;
0, 0, 0, 1;
27.7868, -3.7055, 0.1502, -0.0227;
-60.0327, 38.8112, -0.4534, 0.1779];
B = [0;0;13.7234;-29.6489];
% Calculate the eigenvalues of the A matrix
eigenvalues = eig(A);
% Display the eigenvalues
disp('The eigenvalues of the A matrix are:');
disp(eigenvalues);
% Define the weight matrices Q and R for the LQR controller
Q = diag([100, 100, 100, 100]); % State weighting matrix
R = 1; % Input weighting matrix
% Method 1: Directly compute the LQR gain matrix K
K = lqr(A, B, Q, R);
% % Method 2: solve Riccati equation
G = -B * inv(R)*B';
[P,K1, L1] = icare(A,[], Q, [],[],[],G);
% gain matrix F (=K)
F = inv(R)*B'*P;
Nonlinear system fcn block
function ddq = fcn(dq, q, u)
% Parameters from the problem description
g = 9.81; % gravitational acceleration (m/s^2)
ma = 0.47; % mass of the arm (kg)
la = 0.688; % length of the arm (m)
sa = 0.344; % distance to the center of mass of the arm (m)
Ja = 5.42e-3; % moment of inertia of the arm (kg*m^2)
mp = 0.065; % mass of the pendulum (kg)
Jp = 1.56e-3; % moment of inertia of the pendulum (kg*m^2)
sp = 0.196; % distance to the center of mass of the pendulum (m)
mu_a = 9.29e-3; % damping coefficient of the arm (Nms/rad)
mu_p = 5.23e-4; % damping coefficient of the pendulum (Nms/rad)
% Calculated parameters
J1 = Ja + ma * sa^2 + mp * la^2;
J2 = Jp + mp * sp^2;
Z1 = mp * la * sp;
Z2 = (ma * sa + mp * la) * g;
Z3 = mp * sp * g;
% Extract states
q1 = q(1);
q2 = q(2);
dq1 = dq(1);
dq2 = dq(2);
% Define inertia matrix M
M = [J1, Z1 * cos(q1 - q2);
Z1 * cos(q1 - q2), J2];
% Define damping and centrifugal forces matrix Vm
Vm = [mu_p + mu_a, -mu_p + Z1 * sin(q1 - q2) * dq2;
-mu_p - Z1 * sin(q1 - q2) * dq1, mu_p];
% Define gravitational forces vector G
G = [-Z2 * sin(q1);
-Z3 * sin(q2)];
% Define input matrix B
B = [1; 0];
% Calculate acceleration (ddq) using M*ddq = B*u - Vm*dq - G
ddq = M \ (B * u - Vm * [dq1; dq2] - G);
end
Q1(c)design a LQG controller
Main .m file
% Define system matrices (linearized around the equilibrium point)
A = [0, 0, 1, 0;
0, 0, 0, 1;
27.7868, -3.7055, 0.1502, -0.0227;
-60.0327, 38.8112, -0.4534, 0.1779];
B = [0;0;13.7234; -29.6489];
C = [1 0 0 0; 0 1 0 0]; % Assume we can only measure q1 and q2
D = [0; 0];
Q = diag([100, 100, 100, 100]); % State weighting matrix
R = 1; % Input weighting matrix
K = lqr(A, B, Q, R);
Qe = 0.1 * eye(4); % Process noise covariance
Re = 0.1 * eye(2); % Measurement noise covariance
Ke = lqe(A, Qe, C, Qe, Re); % Compute Kalman gain:
Kalman filter function block
function x_hat_dot = kalman_filter(y, u, x_hat, Ke)
% Define system matrices
A = [0, 0, 1, 0;
0, 0, 0, 1;
27.7868, -3.7055, 0.1502, -0.0227;
-60.0327, 38.8112, -0.4534, 0.1779];
B = [0;0;13.7234;-29.6489];
C = [1 0 0 0; 0 1 0 0]; % Assume we can only measure q1 and q2
% State update equation
x_hat_dot = (A - Ke * C) * x_hat + B * u + Ke * y;
end
Q1(d) LTR
clear;
% Define system matrices (linearized around the equilibrium point)
A = [0, 0, 1, 0;
0, 0, 0, 1;
27.7868, -3.7055, 0.1502, -0.0227;
-60.0327, 38.8112, -0.4534, 0.1779];
B = [0;0;13.7234;-29.6489];
C = [1 0 0 0; 0 1 0 0];
D = [0; 0];
Q = diag([100, 100, 100, 100]); % State weighting matrix
R = 1; % Control input weighting
K = lqr(A, B, Q, R); % Compute LQR state feedback gain using the formula:
% u = -K * x
% Adjusted for LTR Controller Design
Qe = 0.1 * eye(4); % Increase Qe significantly to improve robustness and recover LQR loop properties
Re = 0.1 * eye(2); % Measurement noise covariance (same as before)
% Ke = lqe(A, Qe, C, Qe, Re); % Compute Kalman gain
%
q = 200;
Qq = Qe + q^2 * B * B';
G = -C' * inv(Re) * C;
% solve the Riccati equation
[Pe,ke, L1] = icare(A',[], Qq, [],[],[],G);
Ke = Pe * C' * inv(Re);
Q1. (e) H_infinity state feedback control law
clear;
% Define system matrices (linearized around the equilibrium point)
A = [0, 0, 1, 0;
0, 0, 0, 1;
27.7868, -3.7055, 0.1502, -0.0227;
-60.0327, 38.8112, -0.4534, 0.1779];
B1 = [0, 0,0,0;
0, 0,0,0;
1, 0,0,0;
0, 1,0,0];
B2 = [0;
0;
13.7234;
-29.6489];
C1 = [1, 0, 0, 0;
0, 1, 0, 0;
0, 0, 1, 0;
0, 0, 0, 1;
0, 0, 0, 0];
D12 = [0;
0;
0;
0;
1];
C2 = [1, 0, 0, 0;
0, 1, 0, 0];
D21 = [0,0,1, 0;
0,0,0, 1];
% Riccati equation: A_T * P + P * A + P * (B1 * B1_T / gamma^2 - B2 * B2_T) * P + C1_T * C1 = 0
% Define gamma value
gamma = 100;
G = B1 * B1' / gamma^2 - B2 * B2';
Q = C1'*C1;
% solve the Riccati equation
[P,K1, L1] = icare(A,[], Q, [],[],[],G);
% Display the solution
disp('Riccati equation solution P:');
disp(P);
F = -B2'*P;
% check eigenvalues
judge = A + (B1 * B1'/ gamma^2- B2 * B2') * P;
eigenvalues = eig(judge);
disp('The eigenvalues are:');
disp(eigenvalues);
Q2. (C) LMI solution
clear;
% MATLAB example code for solving xdot=Ax+Bu under constraints P > 0 and
% transpose(A) * P + P * A + P * B * inv(R) * transpose(B) * P + Q < 0, Q > 0
% Defining system, 3rd order system
A = [-2 -1 ; 1 1 ];
B = [1; 0];
rank(ctrb(A, B)) % Check controllability
% Defining SDP variables in LMI
X = sdpvar(2, 2, 'symmetric');
Y = sdpvar(1, 2, 'full');
Q = sdpvar(2, 2, 'symmetric');
% Define LMI structure
LMI1 = X * A' + A * X - B * Y - Y' * B' + Q;
cons = [X >= 0, Q >= 0, LMI1 <= 0];
ops = sdpsettings('solver', 'lmilab'); % Choose a solver with Yalmip
solution = optimize(cons, [], ops); % Solve the LMI
feas = solution.problem;
% If feas = 0 indicates the problem is solvable,
% otherwise the solver cannot find a solution
% Obtain numerical value of the variables
X = value(X);
Y = value(Y);
Q = value(Q);
% Compute controller
K = Y * inv(X);
% Verify the closed-loop system stability
eig(A - B * K)