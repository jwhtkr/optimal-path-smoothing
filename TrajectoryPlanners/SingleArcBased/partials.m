function partials()
%     checkAvoidancePartials()
    checkTerminalPartials()
    % checkVelocityPartials()
    % checkDynamicPartials()
    % checkLogBarrier()
end

function checkAvoidancePartials()
    % Define the variables
    syms q1 q2 q1b q2b d v w theta p3 sig dmax dmin real % Note: 'real' is important, otherwise matlab assumes imaginary
    q = [q1; q2];
    qb = [q1b; q2b];
    d = norm(q-qb);
    
    x = [q1; q2; theta; v; w];
    
    % Define the cost
    L_avoid = p3*(log(dmax) - log(d - dmin));
    
    % Calculate the partial
%     dL_avoid_dd = jacobian(L_avoid, d);
    
    % Calculate partial by hand
    dL_dd = -p3/(d-dmin);
    
    % Look at the error
%     err_dL_avoid = simplify(dL_avoid_dq - dL_dd)
    
    dd_dq = (q - qb)' / d;
    dq_dx = [eye(2), [0; 0], [0; 0], [0; 0]];
    
    dL_dx = dL_dd*dd_dq*dq_dx;
    
    err_dL_dx = simplify(dL_dx - jacobian(L_avoid, x))
    
    
    
    
end

function checkTerminalPartials()
    syms q1 q2 q1d q2d p5 v w theta real
    x = [q1; q2; theta; v; w];
    q = [q1;q2];
    qd = [q1d; q2d];
    
    dphi = p5.*[2*(q-qd)' 0 0 0];
    
    e = q - qd;
    phi = p5*(e'*e);
    
    dphi_syms = jacobian(phi,x);
    err_dphi = simplify(dphi_syms-dphi)
    

end

function checkVelocityPartials()

    % Pretty sure I did this one wrong, I might have confused theta for u
    syms p1 p2 vd v w a al real
    u = [a; al];
       
    dL_du_hand = [p1*2*(v-vd) p2*2*w];
    dL_du_hand = [0 0];
    
    L_vel = p1*(v-vd)^2+p2*w^2;
    
    dL_du_syms = jacobian(L_vel,u);
    err = simplify(dL_du_syms - dL_du_hand)

end

function checkDynamicPartials()
    syms q1 q2 theta v w u a al real
    df_du_hand = [0, 0; 0, 0; 0 0; 1 0; 0 1];
    
    f = [v*cos(theta); v*sin(theta); w; a; al];
    u = [a; al];
    
    df_du_syms = jacobian(f,u);
    
    err_df_du = simplify(df_du_syms - df_du_hand)
    
    df_dx = [0 0 -v*sin(theta) cos(theta) 0; 0 0 v*cos(theta) sin(theta) 0; 0 0 0 0 1; 0 0 0 0 0; 0 0 0 0 0];
    
    x = [q1; q2; theta; v; w];
    
    err_df_dx = simplify(df_dx - jacobian(f,x))
    
end

function checkLogBarrier()
    syms q1 q2 theta qd1 qd2 dmin p3 dmax real
    
    q = [q1;q2];
    qd = [qd1;qd2];
    s = (q-qd)'*(q-qd);
    L_avoid = p3*(log(dmax-dmin)-log(sqrt(s)-dmin));
    dL_avoid_dq = jacobian(L_avoid, q);
    
    dL_ds = -p3*(1/(2*(s-sqrt(s)*dmin)));
    ds_dq = 2*(q-qd)';
    dL_avoid_dq_hand = dL_ds * ds_dq;
    
    err = simplify(dL_avoid_dq - dL_avoid_dq_hand)
    
    syms s real
    dL_avoid = p3*(log(dmax-dmin)-log(sqrt(s)-dmin));
    dL_ds = jacobian(dL_avoid,s);
    dL_ds_hand = (p3/(2*(dmin-s^(1/2))*s^(1/2)));
    
    err = simplify(dL_ds - dL_ds_hand)
    
    
    
    
    
    
end

