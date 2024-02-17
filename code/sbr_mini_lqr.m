pkg load control




%%%%%%%%%%%%%%%%%%%%%%%%%
A = [0 0 1 0;
          0 0 0 1;
          0 -219.3146667 0 0;
          0 133.2775914 0 0]
%%%%%%%%%%%%%%%%%%%%%%%%

B_reduced = [0;
              0;
             14716.1175;
              -3740.075568]

%%%%%%%%%%%%%%%%%%%%%%%%
Mctrb_new = ctrb(A,B_reduced);
rank(Mctrb_new)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];

C_reduced = [0 1 0 0; 0 0 0 1]

D = [0 0;0 0;0 0;0 0];
D_reduced = [0;0;0;0];
D_reduced_2 = [0;0];

#######################################################
Q = [0.1 0 0 0;   # wheel
     0 10 0 0;   # theta
     0 0 2 0;   # wheel_dot
     0 0 0 6];   # theta_dot

R = eye(2);
% R_reduced = 0.012;
R_reduced = 1;

sys = ss(A,B_reduced,C_reduced,D_reduced_2);
k_continuous = lqr(sys,Q,R_reduced)
sys_d = c2d(sys,0.012);            #12ms sampling time
K_discrete = dlqr(sys_d,Q,R_reduced)
eig(sys)

% K_discrete_lqrd = lqrd(A,B_reduced,Q,R_reduced,0.012)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#for Q=eye(4) => K = -0.113806  -0.060363  -0.062918  -0.394597
#for Q=some other => K = -1.7765e-02  -2.7331e-03  -6.3486e-02  -3.0824e-01
#for Q=diag(0.001 131.31 0.001 364.756) => K = -5.383665  -4.722892  -0.062458  -8.21529 #here it oscillatory balanced

As = A - B_reduced*K_discrete;
sys2 = ss(As,B_reduced,C,D_reduced);
#impulse(sys2)
#step(sys2)
