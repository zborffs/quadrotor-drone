classdef QuadrotorParameters
    %QuadrotorParameters is a class whose member variables are the system
    %parameters of the quadmotor model developed ing
    %'gen_quadrotor_dynamics.m' file

    properties
        M % mass of the fuselage (kg)
        m % mass of a rotor (kg)
        R % 'radius' of solid sphere representing inertia of fuselage (m)
        L % length of strut from fuselage to rotor (m)
        l % length of rotor
        g % acceleration due to gravity near surface of the Earth (m/s^2)
        k1 % experimentally determined coeff. representing proportion of angular velocity of rotor that is transferred to force (kg * m)
        k2 % experimentally determined coeff. representing proportion of collective angular momentum of rotors tranferred to torque of body of whole drone (kg m^2 / s)
    end

    methods
        function obj = QuadrotorParameters(M, m, R, L, l, g, k1, k2)
            % simple class constructor where we assign the arguments to the
            % constructor to the associated member variables
            assert(M > 0, "input arg 'M' to QuadrotorParamters constructor must be strictly positive");
            obj.M = M;
            
            assert(m > 0, "input arg 'm' to QuadrotorParamters constructor must be strictly positive");
            obj.m = m;

            assert(R > 0, "input arg 'R' to QuadrotorParamters constructor must be strictly positive");
            obj.R = R;

            assert(L > 0, "input arg 'L' to QuadrotorParamters constructor must be strictly positive");
            obj.L = L;

            assert(l > 0, "input arg 'l' to QuadrotorParamters constructor must be strictly positive");
            obj.l = l;

            assert(g > 0, "input arg 'g' to QuadrotorParamters constructor must be strictly positive");
            obj.g = g;

            assert(k1 > 0, "input arg 'k1' to QuadrotorParamters constructor must be strictly positive");
            obj.k1 = k1;

            assert(k2 > 0, "input arg 'k2' to QuadrotorParamters constructor must be strictly positive");
            obj.k2 = k2;
        end
    end
end