function nlobj = initialise_mpc(L1, L2, Horizon, Ts, maxSteer, x0, u0)   
    nlobj = nlmpc(3, 3, 2);

    nlobj.Ts = Ts;
    nlobj.PredictionHorizon = Horizon;

    nlobj.Model.StateFcn = @(x,u) stateFcn(x,u,L1,L2,Ts);
    nlobj.Model.IsContinuousTime = false;
    nlobj.Model.OutputFcn = @(x,u) x;

    nlobj.MV(1).Min = -0.5;
    nlobj.MV(1).Max = 0.5;
    nlobj.MV(2).Min = -maxSteer;
    nlobj.MV(2).Max = maxSteer;

    nlobj.MV(1).RateMax = 1.26;
    nlobj.MV(2).RateMax = 0.29;

    nlobj.Weights.OutputVariables = [1 1 0.5];
    nlobj.Weights.ManipulatedVariables = [0.1 0.1];
    nlobj.Weights.ManipulatedVariablesRate = [0.05 0.05];

    validateFcns(nlobj,x0,u0);
end

function x_next = stateFcn(x, u, L1, L2, Ts)
    posX = x(1);
    posY = x(2);
    theta = x(3);
    
    v = u(1);
    gamma = u(2);
    
    posX_next = posX + v * cos(theta) * Ts;
    posY_next = posY + v * sin(theta) * Ts;
    theta_dot = (v * sin(gamma)) / (L2 + L1 * cos(gamma));
    theta_next = theta + theta_dot * Ts;
    
    x_next = [posX_next; posY_next; theta_next];
end