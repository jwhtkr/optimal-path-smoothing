% clear all
% close all
% clc

syms q1 q2 psi v w x_des y_des xd_des yd_des xdd_des ydd_des k1 k2 eps real

x = [q1 q2 psi v w]';


Re = [cos(psi) -eps*sin(psi); sin(psi) eps*cos(psi)];
Re_inv = [cos(psi) -sin(psi)/eps; sin(psi) cos(psi)/eps];
w_hat = [0 -w; w 0];
v_bar = [v; w];


q = [q1; q2] + eps*[cos(psi); sin(psi)];
qdot = Re*v_bar;
% qddot = Re*w_hat*v_bar + Re*a_bar
K = [k1 0 k2 0; 0 k1 0 k2];

q_des = [x_des; y_des];
qdot_des = [xd_des; yd_des];
qddot_des = [xdd_des; ydd_des];

u_point = -K*([q; qdot] - [q_des; qdot_des]) + qddot_des;

d1_dx = jacobian(Re_inv*u_point,x);

c = cos(psi);
s = sin(psi);
e = eps;

z1 = (xdd_des - k1*(q1 - x_des + e*c) + k2*(xd_des - v*c + e*w*s));
z2 = (-ydd_des +k1*(q2 - y_des + e*s) + k2*(-yd_des +v*s + e*w*c))/e;
dz1 = k1*e*s + k2*v*s + k2*e*w*c;
dz2 = (k1*e*c + k2*v*c - k2*e*w*s)/e;

d1_dx_hand = [-k1*c, k1*s/e, -s*z1 + c*z2 + c*dz1 + s*dz2, k2*(s^2/e - c^2), k2*(c*s + e*c*s);
              -k1*s,-k1*c/e,  c*z1 + s*z2 + s*dz1 - c*dz2,-k2*(c*s + c*s/e), k2*(e*s^2 - c^2)];
          
err = simplify(d1_dx_hand - d1_dx)

d2_dx = jacobian(w_hat*v_bar,x);

d2_dx_hand = [0 0 0 0 -2*w; 0 0 0 w v];

err = simplify(d2_dx-d2_dx_hand)

u = Re_inv*u_point - w_hat*v_bar;

du_dx = jacobian(u,x);

du_dx_hand = d1_dx_hand - d2_dx_hand;

err = simplify(du_dx - du_dx_hand)
