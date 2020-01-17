classdef ClothoidGenerator < handle
    
    properties
       dt;
       v;
       max_k;
       max_sigma; 
       traj;
       
       clothoid;
       t_span;
       
       circle;
       mu;
       waypoints;
       waypoints_directon;
       max_clothoid_deflection;
       
    end
        
    methods
        function obj = ClothoidGenerator(max_k, v, dt, max_sigma)
            %Creates the full clothoid region from k = 0:k_max
            obj.dt = dt;
            obj.v = v;
            obj.max_k = max_k;
            obj.max_sigma = max_sigma;
            obj.traj = Trajectory2D();
            obj.traj.ds = v*dt;
            s_len = v*(max_k/max_sigma);
            obj.traj.s = 0:obj.traj.ds:s_len;
            obj.traj.cloth_len = length(obj.traj.s);
            obj.traj.v = v*ones(1,s_len);
            obj.traj.a = 0;
            obj.traj.sigma = max_sigma;
            obj.traj.dt = dt;
            obj.traj.s_geo = obj.traj.s;
            
            obj.t_span = obj.traj.s ./ obj.traj.v;
            
            [x, t] = obj.calc_clothoid(v,max_sigma);
            obj.traj.update(x,t);
        end
        
        function [x, t] = calc_clothoid(obj,v,sigma)
            x = zeros(4,1);
            
            % Numerical Integration
            [t_vec,x_vec] = ode45(@(t,x) obj.xdot(x,v,sigma), obj.t_span, x);
            % plot(x_vec(:,1),x_vec(:,2))
            x = x_vec';
            t = t_vec';
        end
        
        function x_dot = xdot(obj,x,v,sigma)
            psi = x(3);
            k = x(4);
            x_dot = [v*cos(psi); v*sin(psi); v*k; sigma];            
        end
        
    end
end