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

  block GantryTrolley_PIDController
    Modelica.Blocks.Interfaces.RealInput setpoint annotation(
      Placement(transformation(origin = {-96, 48}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-80, 8}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealInput feedback annotation(
      Placement(transformation(origin = {-92, -20}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-88, -18}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Math.Add error(k2 = -1) annotation(
      Placement(transformation(origin = {-56, 18}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Gain Proportional(k = Kp)  annotation(
      Placement(transformation(origin = {-10, 56}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.Integrator integrator(k = Ki)  annotation(
      Placement(transformation(origin = {-10, 18}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.Derivative derivative annotation(
      Placement(transformation(origin = {-10, -18}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add3 add3 annotation(
      Placement(transformation(origin = {32, 18}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Interfaces.RealOutputu annotation(
      Placement(transformation(origin = {68, 18}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {68, 18}, extent = {{-10, -10}, {10, 10}})));
  equation
    connect(setpoint, error.u1) annotation(
      Line(points = {{-96, 48}, {-68, 48}, {-68, 24}}, color = {0, 0, 127}));
    connect(feedback, error.u2) annotation(
      Line(points = {{-92, -20}, {-68, -20}, {-68, 12}}, color = {0, 0, 127}));
  connect(integrator.u, error.y) annotation(
      Line(points = {{-22, 18}, {-44, 18}}, color = {0, 0, 127}));
  connect(Proportional.u, error.y) annotation(
      Line(points = {{-22, 56}, {-36, 56}, {-36, 18}, {-44, 18}}, color = {0, 0, 127}));
  connect(derivative.u, error.y) annotation(
      Line(points = {{-22, -18}, {-36, -18}, {-36, 18}, {-44, 18}}, color = {0, 0, 127}));
  connect(integrator.y, add3.u2) annotation(
      Line(points = {{2, 18}, {20, 18}}, color = {0, 0, 127}));
  connect(Proportional.y, add3.u1) annotation(
      Line(points = {{2, 56}, {20, 56}, {20, 26}}, color = {0, 0, 127}));
  connect(derivative.y, add3.u3) annotation(
      Line(points = {{2, -18}, {20, -18}, {20, 10}}, color = {0, 0, 127}));
  end GantryTrolley_PIDController;
  annotation(
    uses(Modelica(version = "4.0.0")));
end BIP_PID_PACKAGE;