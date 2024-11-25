package BIP_PID_PACKAGE
  model GantryTrolley_PlantModel
    // Imports
    import Modelica.Units.SI;
    // Parameters
    parameter SI.Mass m = 0.2 "Mass of pendulum bob's mass";
    parameter SI.Mass M = 10 "Mass of trolley";
    parameter SI.Radius r = 1 "Length of pendulum rope";
    parameter SI.MechanicalImpedance dp = 0.12 "Damping factor for pendulum swing";
    parameter SI.MechanicalImpedance dc = 4.79 "Damping factor for trolley";
    parameter SI.Acceleration g = Modelica.Constants.g_n "Gravitational acceleration";
    // State variables
    SI.Position x "Displacement of trolley";
    SI.Velocity v "Velocity of trolley";
    SI.Angle theta "Angular displacement of pendulum (rad)";
    SI.AngularVelocity omega "Angular velocity of pendulum (rad/s)";
    // Control input
    input SI.Force u "Control signal";
  equation
// Equations based on the provided ODEs
    der(x) = v;
    der(theta) = omega;
    der(v) = (r*(dc*v - m*(g*sin(theta)*cos(theta) + r*sin(theta)*(omega^2)) - u) - (dp*cos(theta)*omega))/(-r*(M + m*sin(theta)^2));
    der(omega) = ((dp*omega*(m + M)) + (m^2*r^2*sin(theta)*cos(theta)*omega^2) + m*r*(g*sin(theta)*(m + M) + (cos(theta)*(u - dc*v))))/(m*r^2*(-M - (m*sin(theta)^2)));
  end GantryTrolley_PlantModel;

  block GantryTrolley_PlantModel_Block
    extends GantryTrolley_PlantModel;
    extends Modelica.Blocks.Icons.Block;
    Modelica.Blocks.Interfaces.RealInput u_input "Input signal connector" annotation(
      Placement(transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealOutput x_output "Output signal connector" annotation(
      Placement(transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}})));
  equation
    u_input = u;
    x_output = x;
  end GantryTrolley_PlantModel_Block;
  
    model GantryTrolley_PIDControllerModel
  parameter Real Kp = 0;
  parameter Real Ki = 0;
  parameter Real Kd = 0;
  equation

  end GantryTrolley_PIDControllerModel;

  block GantryTrolley_PIDController  
    extends GantryTrolley_PIDControllerModel;
    Modelica.Blocks.Interfaces.RealInput setpoint "Input setpoint connector" annotation(
      Placement(transformation(origin = {-120, 50}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealInput feedback "Input feedback connector" annotation(
      Placement(transformation(origin = {-120, -20}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Math.Add error(k2 = -1) annotation(
      Placement(transformation(origin = {-56, 18}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Math.Gain Proportional(k = Kp) annotation(
      Placement(transformation(origin = {-10, 56}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Continuous.Integrator integrator(k = Ki) annotation(
      Placement(transformation(origin = {-10, 18}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Continuous.Derivative derivative(k = Kd)  annotation(
      Placement(transformation(origin = {-10, -18}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Math.Add3 u_output annotation(
      Placement(transformation(origin = {32, 18}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Interfaces.RealOutput u "Output u connector" annotation(
        Placement(transformation(origin = {92, 18}, extent = {{-20, -20}, {20, 20}})));
      //Placement(transformation(origin = {72, 18}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {68, 18}, extent = {{-10, -10}, {10, 10}})));
  equation
    connect(setpoint, error.u1) annotation(
      Line(points = {{-120, 50}, {-70, 50}, {-70, 24}, {-68, 24}}, color = {0, 0, 127}));
    connect(feedback, error.u2) annotation(
      Line(points = {{-120, -20}, {-72, -20}, {-72, 12}, {-68, 12}}, color = {0, 0, 127}));
    connect(integrator.u, error.y) annotation(
      Line(points = {{-22, 18}, {-44, 18}}, color = {0, 0, 127}));
    connect(Proportional.u, error.y) annotation(
      Line(points = {{-22, 56}, {-36, 56}, {-36, 18}, {-44, 18}}, color = {0, 0, 127}));
    connect(derivative.u, error.y) annotation(
      Line(points = {{-22, -18}, {-36, -18}, {-36, 18}, {-44, 18}}, color = {0, 0, 127}));
    connect(integrator.y, u_output.u2) annotation(
      Line(points = {{2, 18}, {20, 18}}, color = {0, 0, 127}));
    connect(Proportional.y, u_output.u1) annotation(
      Line(points = {{2, 56}, {20, 56}, {20, 26}}, color = {0, 0, 127}));
    connect(derivative.y, u_output.u3) annotation(
      Line(points = {{2, -18}, {20, -18}, {20, 10}}, color = {0, 0, 127}));
  connect(u_output.y, u) annotation(
      Line(points = {{44, 18}, {72, 18}}, color = {0, 0, 127}));
  end GantryTrolley_PIDController;

model GantryTrolley_PID_CONTROL_LOOP
GantryTrolley_PlantModel_Block gantryTrolley_PlantModel_Block annotation(
    Placement(transformation(origin = {20, 10}, extent = {{-10, -10}, {10, 10}})));
Modelica.Blocks.Sources.Constant X_Position_Setpoint(k = x_setpoint)  annotation(
    Placement(transformation(origin = {-82, 12}, extent = {{-10, -10}, {10, 10}})));
GantryTrolley_PIDController gantryTrolley_PIDController(Kp = Kp, Ki = Ki, Kd = Kd) annotation(
    Placement(transformation(origin = {-38, 8}, extent = {{-10, -10}, {10, 10}})));

parameter Real x_setpoint = 20 "x_setpoint";
parameter Real Kp = 0 "Proportional Gain";
parameter Real Ki = 0 "Differentational Gain";
parameter Real Kd = 0 "Integrational Gain";

equation
connect(X_Position_Setpoint.y, gantryTrolley_PIDController.setpoint) annotation(
    Line(points = {{-70, 12}, {-50, 12}, {-50, 14}}, color = {0, 0, 127}));
connect(gantryTrolley_PIDController.u, gantryTrolley_PlantModel_Block.u_input) annotation(
    Line(points = {{-28, 10}, {8, 10}}, color = {0, 0, 127}));
connect(gantryTrolley_PlantModel_Block.x_output, gantryTrolley_PIDController.feedback) annotation(
    Line(points = {{32, 10}, {48, 10}, {48, -18}, {-60, -18}, {-60, 6}, {-50, 6}}, color = {0, 0, 127}));
end GantryTrolley_PID_CONTROL_LOOP;
  annotation(
    uses(Modelica(version = "4.0.0")));
end BIP_PID_PACKAGE;