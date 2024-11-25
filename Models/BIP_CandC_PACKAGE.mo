package BIP_CandC_PACKAGE

    model GantryTrolley_PendulumLock
    // Imports
    import Modelica.Units.SI;
    // Parameters
    parameter SI.Mass M = 10 "Mass of trolley";
    parameter SI.MechanicalImpedance dc = 2 "Damping factor for trolley";
    // State variables
    SI.Position x "Displacement of trolley";
    SI.Velocity v "Velocity of trolley";
  equation
    der(x) = v;
    der(v) = (-dc/M)*v;
  end GantryTrolley_PendulumLock;

  model GantryTrolley_TrolleyLock
    // Imports
    import Modelica.Units.SI;
    // Parameters
    parameter SI.Mass m = 0.2 "Mass of pendulum bob's mass";
    parameter SI.Radius r = 1 "Length of pendulum rope";
    parameter SI.MechanicalImpedance dp = 0.5 "Damping factor for pendulum swing";
    parameter SI.Acceleration g = Modelica.Constants.g_n "Gravitational acceleration";
    
    // State variables
    SI.Angle theta "Angular displacement of pendulum (rad)";
    SI.AngularVelocity omega "Angular velocity of pendulum (rad/s)";
  equation
  der(theta) = omega;
    der(omega) = -( (dp*omega) + (m*g*r*sin(theta)) )/(m*r^2);
  end GantryTrolley_TrolleyLock;

  model GantryTrolley_PendulumLockSimulation
    GantryTrolley_PendulumLock gantryTrolley_PendulumLock;
  initial equation
    gantryTrolley_PendulumLock.x = 0;
    gantryTrolley_PendulumLock.v = 5;
  equation
  
  end GantryTrolley_PendulumLockSimulation;

  model GantryTrolley_TrolleyLockSimulation
    GantryTrolley_TrolleyLock gantryTrolley_TrolleyLock;
  initial equation
    gantryTrolley_TrolleyLock.omega = 0;
    gantryTrolley_TrolleyLock.theta = Modelica.Units.Conversions.from_deg(30);
  equation

  end GantryTrolley_TrolleyLockSimulation;
end BIP_CandC_PACKAGE;