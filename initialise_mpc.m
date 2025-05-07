function nlobj = initialise_mpc(L1, L2, Horizon, Ts, maxSteerSpeed, x0, u0)
    % 4 states (x, y, theta, gamma), 4 outputs (zelfde), 2 inputs (v, steer_cmd)
    nlobj = nlmpc(4, 4, 2);

    nlobj.Ts = Ts;
    nlobj.PredictionHorizon = Horizon;

    % State function
    nlobj.Model.StateFcn = @(x,u) stateFcn(x,u,L1,L2,Ts,maxSteerSpeed);
    nlobj.Model.IsContinuousTime = false;

    % Output is gewoon de staat (x, y, theta, gamma)
    nlobj.Model.OutputFcn = @(x,u) x;
    
    % Beperkt de stuurhoek tot 35 graden
    nlobj.States(4).Min = -deg2rad(35);
    nlobj.States(4).Max = deg2rad(35);

    % MV1 = snelheid
    nlobj.MV(1).Min = -0.5;
    nlobj.MV(1).Max = 0.5;
    nlobj.MV(1).RateMax = 1.26;
    nlobj.MV(1).RateMin = -1.26;

    % MV2 = stuurcommando ∈ [-1, 1]
    nlobj.MV(2).Min = -0.5;
    nlobj.MV(2).Max = 0.5;
    % nlobj.MV(2).RateMax = 2;
    % nlobj.MV(2).RateMin = -2;

    % Weights
    nlobj.Weights.OutputVariables = [1 1 0.5 0];  % geen gewicht op gamma
    nlobj.Weights.ManipulatedVariables = [0.1 0.2];
    nlobj.Weights.ManipulatedVariablesRate = [0.1 0.1];

    validateFcns(nlobj, x0, u0);
end


function x_next = stateFcn(x, u, L1, L2, Ts, maxSteerSpeed)
    posX = x(1);
    posY = x(2);
    theta = x(3);
    gamma = x(4);            
    
    v = u(1);                % snelheid
    steer_cmd = u(2);        % stuurcommando -> [-1, 1]

    % Bereken stuurhoeksnelheid
    gamma_dot = steer_cmd * maxSteerSpeed;
    gamma_next = gamma + gamma_dot * Ts;

    % Kinematica
    posX_next = posX + v * cos(theta) * Ts;
    posY_next = posY + v * sin(theta) * Ts;
    theta_dot = (v * sin(gamma)) / (L2 + L1 * cos(gamma));
    theta_next = theta + theta_dot * Ts;

    x_next = [posX_next; posY_next; theta_next; gamma_next];
end
