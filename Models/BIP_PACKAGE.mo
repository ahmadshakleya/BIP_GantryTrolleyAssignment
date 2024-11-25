package BIP_PACKAGE
    model GantryTrolley
      // Imports
      import Modelica.Units.SI;
      // Parameters
      parameter SI.Mass m = 0.2 "Mass of pendulum bob's mass";
      parameter SI.Mass M = 10 "Mass of trolley";
      parameter SI.Radius r = 1 "Length of pendulum rope";
      parameter SI.MechanicalImpedance dp = 0.5 "Damping factor for pendulum swing";
      parameter SI.MechanicalImpedance dc = 2 "Damping factor for trolley";
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
      
  end GantryTrolley;

  class GantryTrolleyDocumentation "Documentation for GantryTrolley Model"
    extends Modelica.Icons.Information;
  
    annotation (DocumentationClass=true, Documentation(info="
    <html>
    <h1>GantryTrolley Model</h1>
    <p>The <code>GantryTrolley</code> model represents a gantry system consisting of a trolley and a pendulum. This model is designed to simulate the dynamics of the system under an impulse-like control input. The equations of motion are derived using the principles of classical mechanics.</p>
    
    <h2>Parameters</h2>
    <ul>
      <li><code>m</code> (SI.Mass): Mass of the pendulum bob (default: 0.2 kg).</li>
      <li><code>M</code> (SI.Mass): Mass of the trolley (default: 10 kg).</li>
      <li><code>r</code> (SI.Radius): Length of the pendulum rope (default: 1 m).</li>
      <li><code>dp</code> (SI.MechanicalImpedance): Damping factor for pendulum swing (default: 0.5 N·s/m).</li>
      <li><code>dc</code> (SI.MechanicalImpedance): Damping factor for trolley motion (default: 2 N·s/m).</li>
      <li><code>g</code> (SI.Acceleration): Gravitational acceleration (default: 9.81 m/s²).</li>
    </ul>
  
    <h2>State Variables</h2>
    <ul>
      <li><code>x</code> (SI.Position): Displacement of the trolley along the track.</li>
      <li><code>v</code> (SI.Velocity): Velocity of the trolley.</li>
      <li><code>theta</code> (SI.Angle): Angular displacement of the pendulum in radians.</li>
      <li><code>omega</code> (SI.AngularVelocity): Angular velocity of the pendulum in rad/s.</li>
    </ul>
  
    <h2>Control Input</h2>
    <ul>
      <li><code>u</code> (SI.Force): Control signal applied to the trolley. An impulse-like control is defined as:
        <blockquote>
          <pre>u = if time < 0.5 then 1000 else 0;</pre>
        </blockquote>
        This applies a force of 1000 N for the first 0.5 seconds, and 0 N afterward.
      </li>
    </ul>
  
    <h2>Equations</h2>
    <p>The following equations govern the motion of the trolley-pendulum system:</p>
    <ul>
      <li><b>Trolley Displacement:</b> <code>der(x) = v;</code></li>
      <li><b>Pendulum Angle:</b> <code>der(theta) = omega;</code></li>
      <li><b>Trolley Velocity:</b>
        <blockquote>
          <pre>
  der(v) = (r * (dc * v - m * (g * sin(theta) * cos(theta) + r * sin(theta) * (omega^2)) - u) - (dp * cos(theta) * omega)) 
           / (-r * (M + m * sin(theta)^2));
          </pre>
        </blockquote>
      </li>
      <li><b>Pendulum Angular Velocity:</b>
        <blockquote>
          <pre>
  der(omega) = ((dp * omega * (m + M)) + (m^2 * r^2 * sin(theta) * cos(theta) * omega^2) + m * r * (g * sin(theta) * (m + M) 
                + (cos(theta) * (u - dc * v)))) / (m * r^2 * (-M - (m * sin(theta)^2)));
          </pre>
        </blockquote>
      </li>
    </ul>
    </html>"));
  end GantryTrolleyDocumentation;

  model StaticGantySimulation
    // Variables
    Real u_input;
    GantryTrolley gantryTrolley(u=u_input);
  initial equation
    gantryTrolley.x = 0;
    gantryTrolley.v = 0;
    gantryTrolley.theta = 0;
    gantryTrolley.omega = 0;
  equation
    u_input = 0;
  end StaticGantySimulation;

  model DynamicGantrySimulation
    // Variables
    Real u_input;
    GantryTrolley gantryTrolley(u=u_input);
  initial equation
    gantryTrolley.x = 0;
    gantryTrolley.v = 0;
    gantryTrolley.theta = 0;
    gantryTrolley.omega = 0;
  equation
    u_input = if time < 0.5 then 1000 else 0;
  end DynamicGantrySimulation;
end BIP_PACKAGE;