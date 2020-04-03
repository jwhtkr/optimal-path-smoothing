classdef OrbitTrajectory < DesiredFlatTrajectory
    %OrbitTrajectory is a class that creates a circular orbit trajectory
    
    properties
        center % 2x1 vector describing the center of the orbit
        rad % scalar > 0 radius of the orbit
        w % Rotational velocity around the orbit
        rw % rad * w
        rw_2 % r*w^2: precomputed
        rw_3 % r*w^3: precomupted
        rw_4 % r*w^4
    end
    
    methods
        function obj = OrbitTrajectory(dt, t0, center, rad, vd)
            %OrbitTrajectory Construct an instance of this class
            %
            % Inputs:
            %   dt: time step size
            %   t0: initial time
            %   center: center of the orbit
            %   rad: radius of the orbit
            %   vd: desired velocity around the orbit
            
            obj = obj@DesiredFlatTrajectory(dt, t0);
            
            % Store input variables
            obj.center = center;
            obj.rad = rad;
            obj.w = vd/obj.rad;            
            
            % Precompute scalings
            obj.rw = obj.rad*obj.w;
            obj.rw_2 = obj.rw*obj.w;
            obj.rw_3 = obj.rw_2*obj.w;
            obj.rw_4 = obj.rw_3*obj.w;
        end
        
        function xd = getDesiredState(obj, t)
        % Returns the desired state
            % Calculate the angle
            psi = obj.w*(t-obj.t0);
            h = [cos(psi); sin(psi)];
            Jh = [0 -1; 1 0] * h;
            
            % Calculate the desired states
            q = obj.center + obj.rad*h;
            qdot = obj.rw*Jh;
            qddot = -obj.rw_2*h;
            qdddot = -obj.rw_3*Jh;
            
            % Output the state
            xd = [q; qdot; qddot; qdddot];
        end
        
        function ud = getDesiredControl(obj, t)
        % Returns the desired control
        
            % Calculate the angle
            psi = obj.w*(t-obj.t0);
            h = [cos(psi); sin(psi)];
            
            % Calculate the fourth derivative of the position
            ud = obj.rw_4*h;
        end
    end
end

