package AERO_model
  package WIND_model
    extends Modelica.Icons.Package;

    model WINDSPEED
      import SI = Modelica.Units.SI;
      import pi = Modelica.Constants.pi;
      Modelica.Blocks.Interfaces.RealOutput Vw annotation(
        Placement(visible = true, transformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {112, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      // Parâmetros constantes:
      parameter Real Vw_constante = 8;
      // Parâmetros da rampa:
      parameter Real Vmax_rampa = 0;
      parameter Real ti_rampa = 5, tf_rampa = 1200;
      // Parâmetros da rajada:
      parameter Real Vmax_rajada = 5;
      parameter Real ti_rajada = 20, tf_rajada = 300;
      SI.Velocity Vw_rampa, Vw_rajada;
    algorithm
// Função rampa:
      if time >= ti_rampa and time < tf_rampa then
        Vw_rampa := Vmax_rampa * (1 - (tf_rampa - time) / (tf_rampa - ti_rampa));
      else
        Vw_rampa := 0;
      end if;
// Função rajada:
      if time >= ti_rajada and time < tf_rajada then
        Vw_rajada := Vmax_rajada / 2 * (1 - cos(2 * pi * (time - ti_rajada) / (tf_rajada - ti_rajada)));
      else
        Vw_rajada := 0;
      end if;
// Soma das parcelas:
      Vw := Vw_constante + Vw_rampa + Vw_rajada;
      annotation(
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-6, Interval = 0.002),
        Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {0, 1}, lineColor = {0, 0, 255}, extent = {{-50, 35}, {50, -35}}, textString = "Vw")}));
    end WINDSPEED;
  end WIND_model;

  package TURBINA_model
    extends Modelica.Icons.Package;

    function CP
      input Real Wtur;
      input Real Vw;
      input Real beta "Pitch angle in degree";
      output Real Cp "Power coefficient";
    protected
      Real lambda, alpha;
      constant Real R = 37.5;
      constant Real C1 = 0.22;
      constant Real C2 = 116;
      constant Real C3 = 0.4;
      constant Real C4 = 0;
      constant Real C5 = 0;
      constant Real C6 = 5;
      constant Real C7 = 12.5;
      constant Real C8 = 0.08;
      constant Real C9 = 0.035;
    algorithm
      lambda := R * Wtur / Vw;
      alpha := 1 / (1 / (lambda + beta * C8) - C9 / (beta ^ 3 + 1));
      Cp := C1 * (C2 / alpha - C3 * beta - C4 * beta ^ C5 - C6) * exp(-C7 / alpha);
    end CP;

    model TURBINA
      import pi = Modelica.Constants.pi;
      // Mechanical parameters of turbine:
      parameter Real Dtur = 0 "Viscous turbine friction" annotation(
        Dialog(group = "Mechanical Data"));
      parameter Real Jtur = 474.5721 "Turbine moment of inertia" annotation(
        Dialog(group = "Mechanical Data"));
      // Aerodynamic parameters  of turbine:
      parameter Real par = 1.225 "Air density" annotation(
        Dialog(group = "Aerodynamic Data"));
      parameter Real R = 37.5 "Helix radius" annotation(
        Dialog(group = "Aerodynamic Data"));
      parameter Real A = pi * R ^ 2 "Propeller area" annotation(
        Dialog(group = "Aerodynamic Data"));
      parameter Real N = 111.5 "Gear ratio" annotation(
        Dialog(group = "Aerodynamic Data"));
      // Parâmetros de controle do beta:
      parameter Real Md_beta = 5 "Max limiter of slew rate" annotation(
        Dialog(group = "Control Data"));
      parameter Real Td_beta = 1 "Constant time of slew rate" annotation(
        Dialog(group = "Control Data"));
      parameter Real ts = 0.01 "Settiling time of pitch angle" annotation(
        Dialog(group = "Control Data"));
      parameter Real zeta = 0.7 "Damping constant of pitch angle" annotation(
        Dialog(group = "Control Data"));
      parameter Real kb = 2 / ts "kb of pitch angle" annotation(
        Dialog(group = "Control Data"));
      parameter Real tb = ts / (8 * zeta ^ 2) "ts of pitch angle" annotation(
        Dialog(group = "Control Data"));
      // Declaração de variáveis:
      Real Wtur, Ttur, Ptur, cp, Teixo;
      Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_Eixo annotation(
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Vw annotation(
        Placement(visible = true, transformation(origin = {-122, -82}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput beta annotation(
        Placement(visible = true, transformation(origin = {-34, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Continuous.TransferFunction transferFunction(a = {tb, 1, kb}, b = {kb}, initType = Modelica.Blocks.Types.Init.SteadyState) annotation(
        Placement(visible = true, transformation(origin = {20, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Nonlinear.SlewRateLimiter slewRateLimiter(Rising = Md_beta, Td = Td_beta) annotation(
        Placement(visible = true, transformation(origin = {54, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    initial equation
      der(Wtur) = 0;
    equation
      Teixo = flange_Eixo.tau;
      Wtur = der(flange_Eixo.phi);
// Convertendo velocidade em conjugado:
      cp = CP(Wtur / N, Vw, transferFunction.y);
      Ptur = 1 / 2 * (par * A * cp * Vw ^ 3);
      Ttur = Ptur / Wtur;
// Equações mecânicas:
      Jtur * der(Wtur) = Ttur + Teixo - Dtur * Wtur;
      connect(beta, transferFunction.u) annotation(
        Line(points = {{-34, 80}, {8, 80}}, color = {0, 0, 127}));
      connect(transferFunction.y, slewRateLimiter.u) annotation(
        Line(points = {{32, 80}, {42, 80}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Ellipse(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{22, -56}, {22, -56}}), Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-3, 4}, lineColor = {0, 0, 255}, extent = {{-75, 30}, {75, -30}}, textString = "TURBINA")}),
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
    end TURBINA;

    model EIXO
      Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation(
        Placement(visible = true, transformation(origin = {-108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b annotation(
        Placement(visible = true, transformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.SpringDamper springDamper(c = Ktm, d = Dtm) annotation(
        Placement(visible = true, transformation(origin = {2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter Real Ktm = 6366.5064 "Shaft elasticity" annotation(
        Dialog(group = "Shaft Data"));
      parameter Real Dtm = 84.4577 "Viscous shaft friction" annotation(
        Dialog(group = "Shaft Data"));
    initial equation
      der(springDamper.flange_a.phi - springDamper.flange_b.phi) = 0;
    equation
      connect(flange_a, springDamper.flange_a) annotation(
        Line(points = {{-108, 0}, {-8, 0}}));
      connect(springDamper.flange_b, flange_b) annotation(
        Line(points = {{12, 0}, {108, 0}}));
      annotation(
        Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 60}, {100, -60}}), Text(origin = {-2, 0}, lineColor = {0, 0, 255}, extent = {{-62, 18}, {62, -18}}, textString = "EIXO")}));
    end EIXO;

    model EOLICA
      AERO_model.TURBINA_model.TURBINA turbina annotation(
        Placement(visible = true, transformation(origin = {-40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      EIXO eixo annotation(
        Placement(visible = true, transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Vw annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput beta annotation(
        Placement(visible = true, transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_Eixo annotation(
        Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(turbina.flange_Eixo, eixo.flange_a) annotation(
        Line(points = {{-28, 0}, {20, 0}}));
      connect(Vw, turbina.Vw) annotation(
        Line(points = {{-120, 0}, {-50, 0}}, color = {0, 0, 127}));
      connect(beta, turbina.beta) annotation(
        Line(points = {{-120, -40}, {-40, -40}, {-40, -10}}, color = {0, 0, 127}));
      connect(eixo.flange_b, flange_Eixo) annotation(
        Line(points = {{42, 0}, {100, 0}}));
      annotation(
        Icon(graphics = {Polygon(origin = {-50, -28}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{50, 24}, {-50, -32}, {10, 30}, {46, 32}, {46, 30}, {50, 24}}), Polygon(origin = {3, 50}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-7, -46}, {-7, 50}, {15, -16}, {1, -46}, {-3, -46}, {-7, -46}}), Rectangle(extent = {{-4, 4}, {4, -4}}), Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, extent = {{-100, 100}, {100, -100}}), Polygon(origin = {42, -19}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-38, 23}, {58, -41}, {-26, -17}, {-42, 15}, {-42, 15}, {-38, 23}}), Ellipse(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-6, 6}, {6, -6}})}));
    end EOLICA;
  end TURBINA_model;

  package CONVERSOR_model
    extends Modelica.Icons.Package;

    model CONVERSOR
      Modelica.Electrical.Polyphase.Sources.SignalCurrent CurrRSC annotation(
        Placement(visible = true, transformation(origin = {-56, 0}, extent = {{-6, 6}, {6, -6}}, rotation = 90)));
      Modelica.Electrical.Analog.Basic.Ground ground annotation(
        Placement(visible = true, transformation(origin = {-56, -36}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = fill(100, 3)) annotation(
        Placement(visible = true, transformation(origin = {-69, 0}, extent = {{-6, -5}, {6, 5}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Basic.Star star annotation(
        Placement(visible = true, transformation(origin = {-56, -24}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Sources.SignalCurrent CurGSC annotation(
        Placement(visible = true, transformation(origin = {54, 0}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
      Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
        Placement(visible = true, transformation(origin = {54, -36}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Basic.Resistor resistor1(R = fill(100, 3)) annotation(
        Placement(visible = true, transformation(origin = {69, 0}, extent = {{-6, 5}, {6, -5}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Basic.Star star1 annotation(
        Placement(visible = true, transformation(origin = {54, -24}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
      Modelica.Electrical.Analog.Basic.Capacitor capacitor(C = 400e-3) annotation(
        Placement(visible = true, transformation(origin = {0, 70}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      Modelica.Electrical.Analog.Basic.Ground ground2 annotation(
        Placement(visible = true, transformation(origin = {0, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Sources.SignalCurrent CurCCRSC annotation(
        Placement(visible = true, transformation(origin = {-58, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Electrical.Analog.Sources.SignalCurrent CurCCGSC annotation(
        Placement(visible = true, transformation(origin = {40, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Electrical.Polyphase.Interfaces.PositivePlug PlugRSC annotation(
        Placement(visible = true, transformation(origin = {-130, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Interfaces.PositivePlug PlugGSC annotation(
        Placement(visible = true, transformation(origin = {138, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput refRSC[3] annotation(
        Placement(visible = true, transformation(origin = {-30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180), iconTransformation(origin = {-60, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealInput refGSC[3] annotation(
        Placement(visible = true, transformation(origin = {30, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor annotation(
        Placement(visible = true, transformation(origin = {-20, 70}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      Modelica.Blocks.Interfaces.RealOutput Vcc annotation(
        Placement(visible = true, transformation(origin = {-30, 32}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {0, -72}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Sensors.PowerSensor PowerRSC annotation(
        Placement(visible = true, transformation(origin = {-90, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sensors.PowerSensor PowerGSC annotation(
        Placement(visible = true, transformation(origin = {84, 6}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    initial equation
      der(capacitor.v) = 0;
    equation
      CurCCRSC.i = PowerRSC.power / Vcc;
      CurCCGSC.i = PowerGSC.power / Vcc;
      connect(ground.p, star.pin_n) annotation(
        Line(points = {{-56, -32}, {-56, -30}}, color = {0, 0, 255}));
      connect(CurrRSC.plug_p, resistor.plug_n) annotation(
        Line(points = {{-56, -6}, {-69, -6}}, color = {0, 0, 255}));
      connect(star.plug_p, CurrRSC.plug_p) annotation(
        Line(points = {{-56, -18}, {-56, -6}}, color = {0, 0, 255}));
      connect(ground1.p, star1.pin_n) annotation(
        Line(points = {{54, -32}, {54, -30}}, color = {0, 0, 255}));
      connect(CurGSC.plug_p, resistor1.plug_n) annotation(
        Line(points = {{54, -6}, {69, -6}}, color = {0, 0, 255}));
      connect(star1.plug_p, CurGSC.plug_p) annotation(
        Line(points = {{54, -18}, {54, -6}}, color = {0, 0, 255}));
      connect(ground2.p, capacitor.n) annotation(
        Line(points = {{0, 46}, {0, 60}}, color = {0, 0, 255}));
      connect(CurrRSC.plug_n, resistor.plug_p) annotation(
        Line(points = {{-56, 6}, {-69, 6}}, color = {0, 0, 255}));
      connect(resistor1.plug_p, CurGSC.plug_n) annotation(
        Line(points = {{69, 6}, {53, 6}}, color = {0, 0, 255}));
      connect(capacitor.p, voltageSensor.p) annotation(
        Line(points = {{0, 80}, {-20, 80}}, color = {0, 0, 255}));
      connect(voltageSensor.n, capacitor.n) annotation(
        Line(points = {{-20, 60}, {0, 60}}, color = {0, 0, 255}));
      connect(CurCCRSC.n, voltageSensor.p) annotation(
        Line(points = {{-58, 80}, {-20, 80}}, color = {0, 0, 255}));
      connect(CurCCRSC.p, voltageSensor.n) annotation(
        Line(points = {{-58, 60}, {-20, 60}}, color = {0, 0, 255}));
      connect(capacitor.p, CurCCGSC.n) annotation(
        Line(points = {{0, 80}, {40, 80}}, color = {0, 0, 255}));
      connect(capacitor.n, CurCCGSC.p) annotation(
        Line(points = {{0, 60}, {40, 60}}, color = {0, 0, 255}));
      connect(resistor.plug_p, PowerRSC.nc) annotation(
        Line(points = {{-69, 6}, {-80, 6}}, color = {0, 0, 255}));
      connect(PlugRSC, PowerRSC.pc) annotation(
        Line(points = {{-130, 6}, {-100, 6}}, color = {0, 0, 255}));
      connect(PowerRSC.nv, star.plug_p) annotation(
        Line(points = {{-90, -4}, {-90, -18}, {-56, -18}}, color = {0, 0, 255}));
      connect(PowerRSC.pv, PowerRSC.pc) annotation(
        Line(points = {{-90, 16}, {-100, 16}, {-100, 6}}));
      connect(PowerGSC.nc, resistor1.plug_p) annotation(
        Line(points = {{74, 6}, {69, 6}}, color = {0, 0, 255}));
      connect(PowerGSC.nv, star1.plug_p) annotation(
        Line(points = {{84, -4}, {84, -18}, {54, -18}}, color = {0, 0, 255}));
      connect(PowerGSC.pv, PowerGSC.pc) annotation(
        Line(points = {{84, 16}, {94, 16}, {94, 6}}, color = {0, 0, 255}));
      connect(CurGSC.i, refGSC) annotation(
        Line(points = {{46.8, 0}, {30, 0}}, color = {0, 0, 127}, thickness = 0.5));
      connect(CurrRSC.i, refRSC) annotation(
        Line(points = {{-48.8, 0}, {-30, 0}}, color = {0, 0, 127}, thickness = 0.5));
      connect(voltageSensor.v, Vcc) annotation(
        Line(points = {{-30, 70}, {-30, 32}}, color = {0, 0, 127}));
      connect(PowerGSC.pc, PlugGSC) annotation(
        Line(points = {{94, 6}, {138, 6}}, color = {0, 0, 255}));
      annotation(
        Icon(graphics = {Rectangle(origin = {-60, 0}, lineColor = {0, 0, 255}, extent = {{-40, 80}, {40, -80}}), Rectangle(origin = {60, 0}, lineColor = {0, 0, 255}, extent = {{-40, 80}, {40, -80}}), Line(origin = {-0.5, 60}, points = {{-19.5, 0}, {20.5, 0}, {18.5, 0}}, color = {0, 0, 255}), Line(origin = {0, -60}, points = {{-20, 0}, {20, 0}, {20, 0}}, color = {0, 0, 255}), Line(origin = {0.5, -33}, points = {{-0.5, -27}, {-0.5, 29}, {-10.5, 29}, {9.5, 29}, {9.5, 29}}, color = {0, 0, 255}), Line(origin = {0.5, 33}, points = {{-0.5, 27}, {-0.5, -29}, {-10.5, -29}, {9.5, -29}, {9.5, -29}}, color = {0, 0, 255}), Text(origin = {-60, 0}, lineColor = {0, 0, 255}, extent = {{-20, 20}, {20, -20}}, textString = "RSC"), Text(origin = {60, 0}, lineColor = {0, 0, 255}, extent = {{-20, 20}, {20, -20}}, textString = "GSC")}));
    end CONVERSOR;
  end CONVERSOR_model;

  package DFIG_model
    extends Modelica.Icons.Package;

    model MIT
      // Libraries of model:
      import SI = Modelica.Units.SI;
      import pi = Modelica.Constants.pi;
      import cm = Modelica.ComplexMath;
      import j = Modelica.ComplexMath.j;
      import MechInterface = Modelica.Mechanics.Rotational.Interfaces;
      import ElecInterface = Modelica.Electrical.Polyphase.Interfaces;
      // Variables of model:
      SI.Angle alpha, theta_r, theta_e, theta_rm;
      SI.AngularFrequency Wr, Wrm;
      SI.Voltage Vas, Vbs, Vcs, Var, Vbr, Vcr, Vqs, Vds, V0s, Vqr, Vdr, V0r;
      SI.Current Iqs, Ids, I0s, Iqr, Idr, I0r, Iar, Ibr, Icr, Ias, Ibs, Ics;
      SI.MagneticFlux fqs, fds, f0s, fqr, fdr, f0r;
      SI.Torque Te, Tm, Ta;
      Complex Ss, Sr, Ssr "VA";
      Real s "%";
      // Constants of model:
      constant SI.AngularFrequency We = 120 * pi;
      // Parameters - DFIG of 2 MW:
      parameter Boolean RP = true "Model initialization" annotation(
        Dialog(group = "Start Data"));
      parameter SI.Resistance rs = 2.381e-3 "Stator resistance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter SI.Resistance rr = 2.381e-3 "Rotor resistance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter SI.Inductance Lls = 6.32e-5 "Stator leakage inductance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter SI.Inductance Llr = 5.04e-5 "Rotor leakage inductance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter SI.Inductance Lm = 1.8944e-3 "Magnetizing inductance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter Integer Polos = 4 "Number of poles" annotation(
        Dialog(group = "Mechanical Data"));
      parameter SI.MomentOfInertia J = 59 "Machine moment of inertia" annotation(
        Dialog(group = "Mechanical Data"));
      parameter SI.RotationalDampingConstant D = 0 "Viscous friction of the machine" annotation(
        Dialog(group = "Mechanical Data"));
      CONTROL_model.tpark Park1;
      CONTROL_model.tpark Park2;
      CONTROL_model.tpark Park3;
      CONTROL_model.tpark Park4;
      ElecInterface.PositivePlug plug_s annotation(
        Placement(visible = true, transformation(origin = {-28, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Interfaces.NegativePlug plug_r annotation(
        Placement(visible = true, transformation(origin = {10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {40, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      MechInterface.Flange_a eixo annotation(
        Placement(visible = true, transformation(origin = {-4, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    initial equation
      if RP then
        theta_e = 0;
        //theta_rm = 0;
        der(Wrm) = 0;
        //der(fqs) = 0;
        //der(fds) = 0;
        //der(f0s) = 0;
        der(fqr) = 0;
        der(fdr) = 0;
        der(f0r) = 0;
      end if;
    equation
//  Conexão dos plugs com as variaveis internas:
      {Vas, Vbs, Vcs} = plug_s.pin[:].v;
      {Ias, Ibs, Ics} = plug_s.pin[:].i;
      {Var, Vbr, Vcr} = plug_r.pin[:].v;
      {Iar, Ibr, Icr} = plug_r.pin[:].i;
//    Conexões mecânicas, com flange:
      Tm = eixo.tau;
      theta_rm = eixo.phi;
//  Equações/relações angulares:
      der(theta_e) = We;
      der(theta_rm) = Wrm;
      theta_r = Polos / 2 * theta_rm;
      s = (We - Wr) / We;
      alpha = theta_e - theta_r;
//  Transformada Park:
      {Vqs, Vds, V0s} = Park1.qd0;
      {Vas, Vbs, Vcs} = Park1.abc;
      theta_e = Park1.theta;
      {Vqr, Vdr, V0r} = Park2.qd0;
      {Var, Vbr, Vcr} = Park2.abc;
      alpha = Park2.theta;
      {Iqs, Ids, I0s} = Park3.qd0;
      {Ias, Ibs, Ics} = Park3.abc;
      theta_e = Park3.theta;
      {Iqr, Idr, I0r} = Park4.qd0;
      {Iar, Ibr, Icr} = Park4.abc;
      alpha = Park4.theta;
//  Estator Fluxo e Tensão:
      fqs = Lls * Iqs + Lm * (Iqs + Iqr);
      fds = Lls * Ids + Lm * (Ids + Idr);
      f0s = Lls * I0s;
      Vqs = rs * Iqs + We * fds + 0*der(fqs);
      Vds = rs * Ids - We * fqs + 0*der(fds);
      V0s = rs * I0s + 0*der(f0s);
//  Rotor Fluxo e Tensão:
      fqr = Llr * Iqr + Lm * (Iqs + Iqr);
      fdr = Llr * Idr + Lm * (Ids + Idr);
      f0r = Llr * I0r;
      Vqr = rr * Iqr + (We - Wr) * fdr + der(fqr);
      Vdr = rr * Idr - (We - Wr) * fqr + der(fdr);
      V0r = rr * I0r + der(f0r);
//  Potências:
      Ss = 3 / 2 * (Vqs - j * Vds) * (Iqs + j * Ids);
      Sr = 3 / 2 * (Vqr - j * Vdr) * (Iqr + j * Idr);
      Ssr = Ss + Sr;
//  Conjugados:
      Te = 3 * Polos / 4 * (fds * Iqs - fqs * Ids);
//  Modelo Mecânico:
      Ta = Tm + Te;
      J * der(Wrm) = (+Tm) + Te - D * Wrm;
      Wr = Polos / 2 * Wrm;
      annotation(
        experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-06, Interval = 0.0001),
        Diagram(coordinateSystem(extent = {{-20, 20}, {20, -20}})),
        Icon(graphics = {Bitmap(extent = {{20, 0}, {20, 0}}), Polygon(origin = {10, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-70, 60}, {-90, 40}, {-90, -40}, {-70, -60}, {70, -60}, {90, -40}, {90, 40}, {70, 60}, {-70, 60}, {-70, 60}}), Rectangle(origin = {-90, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, extent = {{-10, 8}, {10, -8}}), Rectangle(origin = {10, 70}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-30, 10}, {30, -10}}), Rectangle(origin = {40, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-24, 22}, {24, -22}}), Rectangle(origin = {40, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, extent = {{-24, 8}, {24, -8}}), Line(origin = {44.32, -37.76}, points = {{-2.19338, -22.7774}, {-2.19338, 23.2226}, {-2.19338, 23.2226}}, color = {0, 0, 255}), Line(origin = {51.13, -36.57}, points = {{5, 23}, {5, -15}, {5, -23}, {5, -23}, {5, -23}}, color = {0, 0, 255}), Line(origin = {33.23, -36.73}, points = {{-5, 23}, {-5, -15}, {-5, -23}, {-5, -23}, {-5, -23}}, color = {0, 0, 255}), Rectangle(origin = {42, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-4, 14}, {4, -14}}), Rectangle(origin = {56, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-4, 14}, {4, -14}}), Rectangle(origin = {28, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-4, 14}, {4, -14}}), Line(origin = {-48.82, 0.06}, points = {{-11.1784, 59.9389}, {-11.1784, -60.0611}, {-11.1784, -60.0611}}, color = {0, 0, 255}), Line(origin = {80, 0}, points = {{0, 60}, {0, -60}, {0, -60}}, color = {0, 0, 255}), Rectangle(origin = {40, -67}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-20, 7}, {20, -7}}), Line(origin = {10, 0}, points = {{-90, -40}, {-90, 40}, {-70, 60}, {70, 60}, {90, 40}, {90, -40}, {70, -60}, {-70, -60}, {-90, -40}, {-90, -40}}, color = {0, 0, 255}), Text(origin = {-24, 3}, lineColor = {0, 0, 255}, extent = {{-36, 19}, {36, -19}}, textString = "DFIG")}));
    end MIT;

    model MIT_complet
      // Libraries of model:
      import SI = Modelica.Units.SI;
      import pi = Modelica.Constants.pi;
      import cm = Modelica.ComplexMath;
      import j = Modelica.ComplexMath.j;
      import MechInterface = Modelica.Mechanics.Rotational.Interfaces;
      import ElecInterface = Modelica.Electrical.Polyphase.Interfaces;
      // Variables of model:
      SI.Angle alpha, theta_r, theta_e, theta_rm;
      SI.AngularFrequency Wr, Wrm;
      SI.Voltage Vas, Vbs, Vcs, Var, Vbr, Vcr, Vqs, Vds, V0s, Vqr, Vdr, V0r;
      SI.Current Iqs, Ids, I0s, Iqr, Idr, I0r, Iar, Ibr, Icr, Ias, Ibs, Ics;
      SI.MagneticFlux fqs, fds, f0s, fqr, fdr, f0r;
      SI.Torque Te, Tm, Ta;
      Complex Ss, Sr, Ssr "VA";
      Real s "%";
      // Constants of model:
      constant SI.AngularFrequency We = 120 * pi;
      // Parameters - DFIG of 2 MW:
      parameter Boolean RP = true "Model initialization" annotation(
        Dialog(group = "Start Data"));
      parameter SI.Resistance rs = 2.381e-3 "Stator resistance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter SI.Resistance rr = 2.381e-3 "Rotor resistance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter SI.Inductance Lls = 6.32e-5 "Stator leakage inductance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter SI.Inductance Llr = 5.04e-5 "Rotor leakage inductance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter SI.Inductance Lm = 1.8944e-3 "Magnetizing inductance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter Integer Polos = 4 "Number of poles" annotation(
        Dialog(group = "Mechanical Data"));
      parameter SI.MomentOfInertia J = 59 "Machine moment of inertia" annotation(
        Dialog(group = "Mechanical Data"));
      parameter SI.RotationalDampingConstant D = 0 "Viscous friction of the machine" annotation(
        Dialog(group = "Mechanical Data"));
      CONTROL_model.tpark Park1;
      CONTROL_model.tpark Park2;
      CONTROL_model.tpark Park3;
      CONTROL_model.tpark Park4;
      ElecInterface.PositivePlug plug_s annotation(
        Placement(visible = true, transformation(origin = {-28, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Interfaces.NegativePlug plug_r annotation(
        Placement(visible = true, transformation(origin = {10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {40, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      MechInterface.Flange_a eixo annotation(
        Placement(visible = true, transformation(origin = {-4, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    initial equation
      if RP then
        theta_e = 0;
        //theta_rm = 0;
        der(Wrm) = 0;
        der(fqs) = 0;
        der(fds) = 0;
        der(f0s) = 0;
        der(fqr) = 0;
        der(fdr) = 0;
        der(f0r) = 0;
      end if;
    equation
//  Conexão dos plugs com as variaveis internas:
      {Vas, Vbs, Vcs} = plug_s.pin[:].v;
      {Ias, Ibs, Ics} = plug_s.pin[:].i;
      {Var, Vbr, Vcr} = plug_r.pin[:].v;
      {Iar, Ibr, Icr} = plug_r.pin[:].i;
//    Conexões mecânicas, com flange:
      Tm = eixo.tau;
      theta_rm = eixo.phi;
//  Equações/relações angulares:
      der(theta_e) = We;
      der(theta_rm) = Wrm;
      theta_r = Polos / 2 * theta_rm;
      s = (We - Wr) / We;
      alpha = theta_e - theta_r;
//  Transformada Park:
      {Vqs, Vds, V0s} = Park1.qd0;
      {Vas, Vbs, Vcs} = Park1.abc;
      theta_e = Park1.theta;
      {Vqr, Vdr, V0r} = Park2.qd0;
      {Var, Vbr, Vcr} = Park2.abc;
      alpha = Park2.theta;
      {Iqs, Ids, I0s} = Park3.qd0;
      {Ias, Ibs, Ics} = Park3.abc;
      theta_e = Park3.theta;
      {Iqr, Idr, I0r} = Park4.qd0;
      {Iar, Ibr, Icr} = Park4.abc;
      alpha = Park4.theta;
//  Estator Fluxo e Tensão:
      fqs = Lls * Iqs + Lm * (Iqs + Iqr);
      fds = Lls * Ids + Lm * (Ids + Idr);
      f0s = Lls * I0s;
      Vqs = rs * Iqs + We * fds + der(fqs);
      Vds = rs * Ids - We * fqs + der(fds);
      V0s = rs * I0s + der(f0s);
//  Rotor Fluxo e Tensão:
      fqr = Llr * Iqr + Lm * (Iqs + Iqr);
      fdr = Llr * Idr + Lm * (Ids + Idr);
      f0r = Llr * I0r;
      Vqr = rr * Iqr + (We - Wr) * fdr + der(fqr);
      Vdr = rr * Idr - (We - Wr) * fqr + der(fdr);
      V0r = rr * I0r + der(f0r);
//  Potências:
      Ss = 3 / 2 * (Vqs - j * Vds) * (Iqs + j * Ids);
      Sr = 3 / 2 * (Vqr - j * Vdr) * (Iqr + j * Idr);
      Ssr = Ss + Sr;
//  Conjugados:
      Te = 3 * Polos / 4 * (fds * Iqs - fqs * Ids);
//  Modelo Mecânico:
      Ta = Tm + Te;
      J * der(Wrm) = (+Tm) + Te - D * Wrm;
      Wr = Polos / 2 * Wrm;
      annotation(
        experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-06, Interval = 0.0001),
        Diagram(coordinateSystem(extent = {{-20, 20}, {20, -20}})),
        Icon(graphics = {Bitmap(extent = {{20, 0}, {20, 0}}), Polygon(origin = {10, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-70, 60}, {-90, 40}, {-90, -40}, {-70, -60}, {70, -60}, {90, -40}, {90, 40}, {70, 60}, {-70, 60}, {-70, 60}}), Rectangle(origin = {-90, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, extent = {{-10, 8}, {10, -8}}), Rectangle(origin = {10, 70}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-30, 10}, {30, -10}}), Rectangle(origin = {40, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-24, 22}, {24, -22}}), Rectangle(origin = {40, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, extent = {{-24, 8}, {24, -8}}), Line(origin = {44.32, -37.76}, points = {{-2.19338, -22.7774}, {-2.19338, 23.2226}, {-2.19338, 23.2226}}, color = {0, 0, 255}), Line(origin = {51.13, -36.57}, points = {{5, 23}, {5, -15}, {5, -23}, {5, -23}, {5, -23}}, color = {0, 0, 255}), Line(origin = {33.23, -36.73}, points = {{-5, 23}, {-5, -15}, {-5, -23}, {-5, -23}, {-5, -23}}, color = {0, 0, 255}), Rectangle(origin = {42, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-4, 14}, {4, -14}}), Rectangle(origin = {56, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-4, 14}, {4, -14}}), Rectangle(origin = {28, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-4, 14}, {4, -14}}), Line(origin = {-48.82, 0.06}, points = {{-11.1784, 59.9389}, {-11.1784, -60.0611}, {-11.1784, -60.0611}}, color = {0, 0, 255}), Line(origin = {80, 0}, points = {{0, 60}, {0, -60}, {0, -60}}, color = {0, 0, 255}), Rectangle(origin = {40, -67}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-20, 7}, {20, -7}}), Line(origin = {10, 0}, points = {{-90, -40}, {-90, 40}, {-70, 60}, {70, 60}, {90, 40}, {90, -40}, {70, -60}, {-70, -60}, {-90, -40}, {-90, -40}}, color = {0, 0, 255}), Text(origin = {-24, 3}, lineColor = {0, 0, 255}, extent = {{-36, 19}, {36, -19}}, textString = "DFIG")}));
    end MIT_complet;

    model MIT_semderRotor
      // Libraries of model:
      import SI = Modelica.Units.SI;
      import pi = Modelica.Constants.pi;
      import cm = Modelica.ComplexMath;
      import j = Modelica.ComplexMath.j;
      import MechInterface = Modelica.Mechanics.Rotational.Interfaces;
      import ElecInterface = Modelica.Electrical.Polyphase.Interfaces;
      // Variables of model:
      SI.Angle alpha, theta_r, theta_e, theta_rm;
      SI.AngularFrequency Wr, Wrm;
      SI.Voltage Vas, Vbs, Vcs, Var, Vbr, Vcr, Vqs, Vds, V0s, Vqr, Vdr, V0r;
      SI.Current Iqs, Ids, I0s, Iqr, Idr, I0r, Iar, Ibr, Icr, Ias, Ibs, Ics;
      SI.MagneticFlux fqs, fds, f0s, fqr, fdr, f0r;
      SI.Torque Te, Tm, Ta;
      Complex Ss, Sr, Ssr "VA";
      Real s "%";
      // Constants of model:
      constant SI.AngularFrequency We = 120 * pi;
      // Parameters - DFIG of 2 MW:
      parameter Boolean RP = true "Model initialization" annotation(
        Dialog(group = "Start Data"));
      parameter SI.Resistance rs = 2.381e-3 "Stator resistance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter SI.Resistance rr = 2.381e-3 "Rotor resistance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter SI.Inductance Lls = 6.32e-5 "Stator leakage inductance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter SI.Inductance Llr = 5.04e-5 "Rotor leakage inductance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter SI.Inductance Lm = 1.8944e-3 "Magnetizing inductance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter Integer Polos = 4 "Number of poles" annotation(
        Dialog(group = "Mechanical Data"));
      parameter SI.MomentOfInertia J = 59 "Machine moment of inertia" annotation(
        Dialog(group = "Mechanical Data"));
      parameter SI.RotationalDampingConstant D = 0 "Viscous friction of the machine" annotation(
        Dialog(group = "Mechanical Data"));
      CONTROL_model.tpark Park1;
      CONTROL_model.tpark Park2;
      CONTROL_model.tpark Park3;
      CONTROL_model.tpark Park4;
      ElecInterface.PositivePlug plug_s annotation(
        Placement(visible = true, transformation(origin = {-28, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Interfaces.NegativePlug plug_r annotation(
        Placement(visible = true, transformation(origin = {10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {40, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      MechInterface.Flange_a eixo annotation(
        Placement(visible = true, transformation(origin = {-4, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    initial equation
      if RP then
        theta_e = 0;
        //theta_rm = 0;
        der(Wrm) = 0;
        der(fqs) = 0;
        der(fds) = 0;
        der(f0s) = 0;
        //der(fqr) = 0;
        //der(fdr) = 0;
        //der(f0r) = 0;
      end if;
    equation
//  Conexão dos plugs com as variaveis internas:
      {Vas, Vbs, Vcs} = plug_s.pin[:].v;
      {Ias, Ibs, Ics} = plug_s.pin[:].i;
      {Var, Vbr, Vcr} = plug_r.pin[:].v;
      {Iar, Ibr, Icr} = plug_r.pin[:].i;
//    Conexões mecânicas, com flange:
      Tm = eixo.tau;
      theta_rm = eixo.phi;
//  Equações/relações angulares:
      der(theta_e) = We;
      der(theta_rm) = Wrm;
      theta_r = Polos / 2 * theta_rm;
      s = (We - Wr) / We;
      alpha = theta_e - theta_r;
//  Transformada Park:
      {Vqs, Vds, V0s} = Park1.qd0;
      {Vas, Vbs, Vcs} = Park1.abc;
      theta_e = Park1.theta;
      {Vqr, Vdr, V0r} = Park2.qd0;
      {Var, Vbr, Vcr} = Park2.abc;
      alpha = Park2.theta;
      {Iqs, Ids, I0s} = Park3.qd0;
      {Ias, Ibs, Ics} = Park3.abc;
      theta_e = Park3.theta;
      {Iqr, Idr, I0r} = Park4.qd0;
      {Iar, Ibr, Icr} = Park4.abc;
      alpha = Park4.theta;
//  Estator Fluxo e Tensão:
      fqs = Lls * Iqs + Lm * (Iqs + Iqr);
      fds = Lls * Ids + Lm * (Ids + Idr);
      f0s = Lls * I0s;
      Vqs = rs * Iqs + We * fds + der(fqs);
      Vds = rs * Ids - We * fqs + der(fds);
      V0s = rs * I0s + der(f0s);
//  Rotor Fluxo e Tensão:
      fqr = Llr * Iqr + Lm * (Iqs + Iqr);
      fdr = Llr * Idr + Lm * (Ids + Idr);
      f0r = Llr * I0r;
      Vqr = rr * Iqr + (We - Wr) * fdr + 0 * der(fqr);
      Vdr = rr * Idr - (We - Wr) * fqr + 0 * der(fdr);
      V0r = rr * I0r + 0*der(f0r);
//  Potências:
      Ss = 3 / 2 * (Vqs - j * Vds) * (Iqs + j * Ids);
      Sr = 3 / 2 * (Vqr - j * Vdr) * (Iqr + j * Idr);
      Ssr = Ss + Sr;
//  Conjugados:
      Te = 3 * Polos / 4 * (fds * Iqs - fqs * Ids);
//  Modelo Mecânico:
      Ta = Tm + Te;
      J * der(Wrm) = (+Tm) + Te - D * Wrm;
      Wr = Polos / 2 * Wrm;
      annotation(
        experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-06, Interval = 0.0001),
        Diagram(coordinateSystem(extent = {{-20, 20}, {20, -20}})),
        Icon(graphics = {Bitmap(extent = {{20, 0}, {20, 0}}), Polygon(origin = {10, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-70, 60}, {-90, 40}, {-90, -40}, {-70, -60}, {70, -60}, {90, -40}, {90, 40}, {70, 60}, {-70, 60}, {-70, 60}}), Rectangle(origin = {-90, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, extent = {{-10, 8}, {10, -8}}), Rectangle(origin = {10, 70}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-30, 10}, {30, -10}}), Rectangle(origin = {40, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-24, 22}, {24, -22}}), Rectangle(origin = {40, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, extent = {{-24, 8}, {24, -8}}), Line(origin = {44.32, -37.76}, points = {{-2.19338, -22.7774}, {-2.19338, 23.2226}, {-2.19338, 23.2226}}, color = {0, 0, 255}), Line(origin = {51.13, -36.57}, points = {{5, 23}, {5, -15}, {5, -23}, {5, -23}, {5, -23}}, color = {0, 0, 255}), Line(origin = {33.23, -36.73}, points = {{-5, 23}, {-5, -15}, {-5, -23}, {-5, -23}, {-5, -23}}, color = {0, 0, 255}), Rectangle(origin = {42, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-4, 14}, {4, -14}}), Rectangle(origin = {56, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-4, 14}, {4, -14}}), Rectangle(origin = {28, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-4, 14}, {4, -14}}), Line(origin = {-48.82, 0.06}, points = {{-11.1784, 59.9389}, {-11.1784, -60.0611}, {-11.1784, -60.0611}}, color = {0, 0, 255}), Line(origin = {80, 0}, points = {{0, 60}, {0, -60}, {0, -60}}, color = {0, 0, 255}), Rectangle(origin = {40, -67}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-20, 7}, {20, -7}}), Line(origin = {10, 0}, points = {{-90, -40}, {-90, 40}, {-70, 60}, {70, 60}, {90, 40}, {90, -40}, {70, -60}, {-70, -60}, {-90, -40}, {-90, -40}}, color = {0, 0, 255}), Text(origin = {-24, 3}, lineColor = {0, 0, 255}, extent = {{-36, 19}, {36, -19}}, textString = "DFIG")}));
    end MIT_semderRotor;
  end DFIG_model;

  package CONTROL_model
    extends Modelica.Icons.Package;

    model CONTROL_MAQ
      // Parameters od control:
      parameter Real Htotal = 474.5721 + 59 annotation(
        Dialog(group = "Mechanical Data"));
      parameter Real Dtotal = 0 annotation(
        Dialog(group = "Mechanical Data"));
      parameter Real Ceq = 400e-3 annotation(
        Dialog(group = "Converter Data"));
      parameter Real KC = -3 / 2 * 690 / 1400 / Ceq annotation(
        Dialog(group = "Converter Data"));
      // Control setings:
      parameter Real zetaWrm = 1 annotation(
        Dialog(group = "Control Data"));
      parameter Real zetaVcc = 1 annotation(
        Dialog(group = "Control Data"));
      parameter Real tsWrm = 3 annotation(
        Dialog(group = "Control Data"));
      parameter Real tsVcc = 3 annotation(
        Dialog(group = "Control Data"));
      // Constant control calculed:
      parameter Real Kp_Wrm = 8 * (2 * Htotal) / tsWrm - Dtotal annotation(
        Dialog(group = "Control Data"));
      parameter Real Ki_Wrm = 16 * (2 * Htotal) / (zetaWrm * tsWrm) ^ 2 annotation(
        Dialog(group = "Control Data"));
      parameter Real Ki_Qg = 1, Kp_Qg = 0 annotation(
        Dialog(group = "Control Data"));
      parameter Real Kp_Vcc = 2 * zetaVcc * 3770 / KC annotation(
        Dialog(group = "Control Calculed"));
      parameter Real Ki_Vcc = 3770 ^ 2 / KC annotation(
        Dialog(group = "Control Calculed"));
      AERO_model.CONTROL_model.CONTROL_RSC control_rsc annotation(
        Placement(visible = true, transformation(origin = {-50, -10}, extent = {{-30, -30}, {30, 30}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput iabcr[3] annotation(
        Placement(visible = true, transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      AERO_model.CONTROL_model.PLL pll annotation(
        Placement(visible = true, transformation(origin = {-2.22045e-16, -70}, extent = {{-20, 20}, {20, -20}}, rotation = 90)));
      Modelica.Electrical.Polyphase.Interfaces.PositivePlug Vsmed annotation(
        Placement(visible = true, transformation(origin = {0, -120}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Wrmmed annotation(
        Placement(visible = true, transformation(origin = {-110, -68}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Wrmref annotation(
        Placement(visible = true, transformation(origin = {-90, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Qgmed annotation(
        Placement(visible = true, transformation(origin = {-63, -119}, extent = {{-9, -9}, {9, 9}}, rotation = 0), iconTransformation(origin = {110, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      Modelica.Blocks.Interfaces.RealInput Qgref annotation(
        Placement(visible = true, transformation(origin = {-76, -102}, extent = {{-8, -8}, {8, 8}}, rotation = 0), iconTransformation(origin = {110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      AERO_model.CONTROL_model.CONTROL_GSC control_gsc annotation(
        Placement(visible = true, transformation(origin = {50, -10}, extent = {{-30, -30}, {30, 30}}, rotation = 0)));
      AERO_model.CONTROL_model.PI_Astrom pI_Astrom(ki = Ki_Wrm, kp = Kp_Wrm) annotation(
        Placement(visible = true, transformation(origin = {-76, -68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      AERO_model.CONTROL_model.PI_Astrom pI_Astrom1(ki = Ki_Qg, kp = 0) annotation(
        Placement(visible = true, transformation(origin = {-52, -102}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      AERO_model.CONTROL_model.PI_Astrom pI_Astrom2(ki = Ki_Vcc, kp = Kp_Vcc) annotation(
        Placement(visible = true, transformation(origin = {80, -66}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput iabcg[3] annotation(
        Placement(visible = true, transformation(origin = {50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 90), iconTransformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealInput Vccmed annotation(
        Placement(visible = true, transformation(origin = {120, -66}, extent = {{8, -8}, {-8, 8}}, rotation = 0), iconTransformation(origin = {0, 70}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
      Modelica.Blocks.Sources.Constant const(k = 1400 * 1400) annotation(
        Placement(visible = true, transformation(origin = {94, -88}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Product product annotation(
        Placement(visible = true, transformation(origin = {102, -66}, extent = {{6, -6}, {-6, 6}}, rotation = 0)));
      Modelica.Blocks.Sources.Constant constant1(k = 0) annotation(
        Placement(visible = true, transformation(origin = {50, -100}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    equation
      connect(control_rsc.iabcr, iabcr) annotation(
        Line(points = {{-50, 23}, {-50, 90}}, color = {0, 0, 127}, thickness = 0.5));
      connect(control_rsc.thetaE_PLL, pll.ThetaPll) annotation(
        Line(points = {{-17, -34}, {-12, -34}, {-12, -48}}, color = {0, 0, 127}));
      connect(control_rsc.We_PLL, pll.WePll) annotation(
        Line(points = {{-17, -28}, {0, -28}, {0, -48}}, color = {0, 0, 127}));
      connect(control_rsc.Vqs_PLL, pll.VqPll) annotation(
        Line(points = {{-17, -22}, {12, -22}, {12, -48}}, color = {0, 0, 127}));
      connect(pll.vabc, Vsmed) annotation(
        Line(points = {{0, -92}, {0, -120}}, color = {0, 0, 255}));
      connect(Wrmmed, control_rsc.Wrm) annotation(
        Line(points = {{-110, -68}, {-94, -68}, {-94, -30}, {-82, -30}}, color = {0, 0, 127}));
      connect(pI_Astrom.y, control_rsc.Te_esp) annotation(
        Line(points = {{-64, -68}, {-62, -68}, {-62, -42}}, color = {0, 0, 127}));
      connect(pI_Astrom1.y, control_rsc.Qs_esp) annotation(
        Line(points = {{-41, -102}, {-38, -102}, {-38, -42}}, color = {0, 0, 127}));
      connect(Wrmmed, pI_Astrom.m) annotation(
        Line(points = {{-110, -68}, {-86, -68}}, color = {0, 0, 127}));
      connect(Qgref, pI_Astrom1.m) annotation(
        Line(points = {{-76, -102}, {-63, -102}}, color = {0, 0, 127}));
      connect(Qgmed, pI_Astrom1.r) annotation(
        Line(points = {{-63, -119}, {-52, -119}, {-52, -113}}, color = {0, 0, 127}));
      connect(pI_Astrom.r, Wrmref) annotation(
        Line(points = {{-76, -78}, {-76, -85}, {-90, -85}, {-90, -84}}, color = {0, 0, 127}));
      connect(iabcg, control_gsc.iabcg) annotation(
        Line(points = {{50, 90}, {50, 24}}, color = {0, 0, 127}, thickness = 0.5));
      connect(const.y, pI_Astrom2.r) annotation(
        Line(points = {{83, -88}, {80, -88}, {80, -76}}, color = {0, 0, 127}));
      connect(pI_Astrom2.m, product.y) annotation(
        Line(points = {{92, -66}, {96, -66}}, color = {0, 0, 127}));
      connect(product.u1, Vccmed) annotation(
        Line(points = {{110, -62}, {120, -62}, {120, -66}}, color = {0, 0, 127}));
      connect(product.u2, Vccmed) annotation(
        Line(points = {{110, -70}, {120, -70}, {120, -66}}, color = {0, 0, 127}));
      connect(pI_Astrom2.y, control_gsc.Vcc_esp) annotation(
        Line(points = {{70, -66}, {66, -66}, {66, -42}}, color = {0, 0, 127}));
      connect(constant1.y, control_gsc.Idg_esp) annotation(
        Line(points = {{40, -100}, {36, -100}, {36, -42}}, color = {0, 0, 127}));
      connect(control_gsc.theta_PLL, pll.ThetaPll) annotation(
        Line(points = {{18, -34}, {-12, -34}, {-12, -48}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 60}, {100, -60}}), Text(origin = {4, -1}, lineColor = {0, 0, 255}, extent = {{-58, 23}, {58, -23}}, textString = "CTRL MACHINE")}));
    end CONTROL_MAQ;

    model CRTL_TUR
      Modelica.Blocks.Interfaces.RealInput Vw annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-112, 1.77636e-15}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput beta annotation(
        Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealOutput Wrm_opt annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter Real R = 37.5 "Helix radius" annotation(
        Dialog(group = "Turbine Data"));
      parameter Real N = 111.5 "Gear ratio" annotation(
        Dialog(group = "Turbine Data"));
      parameter Real LBD_opt = 6.3279 "Tip speed ratio optimal" annotation(
        Dialog(group = "Turbine Data"));
      parameter Real Vw_min = 4 "Min wind speed" annotation(
        Dialog(group = "Turbine Data"));
      parameter Real Vw_nom = 12 "Nom wind speed" annotation(
        Dialog(group = "Turbine Data"));
      parameter Real Vw_max = 25 "Max wind speed" annotation(
        Dialog(group = "Turbine Data"));
      parameter Real Vw_wmin = 8.071353643090237 "Min wind for rotor speed" annotation(
        Dialog(group = "Turbine Data"));
      parameter Real Vw_wmax = 11.175720428894175 "Max wind for rotor speed" annotation(
        Dialog(group = "Turbine Data"));
      Modelica.Blocks.Tables.CombiTable1Ds combiTable1Ds(extrapolation = Modelica.Blocks.Types.Extrapolation.LastTwoPoints, fileName = "/home/uemura/MYCODE/TCC/Simu3/mybeta.mat", tableName = "beta", tableOnFile = true, verboseRead = false) annotation(
        Placement(visible = true, transformation(origin = {50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    algorithm
      if Vw >= Vw_min and Vw < Vw_wmin then
        Wrm_opt := N * LBD_opt * Vw_wmin / R;
        beta := 0;
      elseif Vw >= Vw_wmin and Vw < Vw_wmax then
        Wrm_opt := N * LBD_opt * Vw / R;
        beta := 0;
      elseif Vw >= Vw_wmax and Vw < Vw_nom then
        Wrm_opt := N * LBD_opt * Vw_wmax / R;
        beta := 0;
      elseif Vw >= Vw_nom and Vw <= Vw_max then
        Wrm_opt := N * LBD_opt * Vw_wmax / R;
        combiTable1Ds.u := Vw - 12;
        beta := combiTable1Ds.y[1];
      end if;
      annotation(
        Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 60}, {100, -60}}), Text(origin = {4, -1}, lineColor = {0, 0, 255}, extent = {{-66, 45}, {66, -45}}, textString = "CTRL TURBINE")}),
        experiment(StartTime = 0, StopTime = 510, Tolerance = 1e-06, Interval = 0.005));
    end CRTL_TUR;

    model CONTROL_RSC
      import SI = Modelica.Units.SI;
      import pi = Modelica.Constants.pi;
      SI.Current Iqr, Idr;
      SI.Angle thetaR, alpha;
      CONTROL_model.tpark Park1;
      //  Parâmetros MIT 2MW
      parameter SI.Inductance Lls = 6.32e-5 "Stator leakage inductance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter SI.Inductance Lm = 1.8944e-3 "Magnetizing inductance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter Integer Polos = 4 "Number of poles" annotation(
        Dialog(group = "Mechanical Data"));
      Modelica.Blocks.Interfaces.RealInput Wrm annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput iabcr[3] annotation(
        Placement(visible = true, transformation(origin = {120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {4.44089e-16, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealInput Vqs_PLL annotation(
        Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      Modelica.Blocks.Interfaces.RealInput thetaE_PLL annotation(
        Placement(visible = true, transformation(origin = {-120, -52}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      Modelica.Blocks.Interfaces.RealInput Te_esp annotation(
        Placement(visible = true, transformation(origin = {-50, -74}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-50, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealInput Qs_esp annotation(
        Placement(visible = true, transformation(origin = {30, -82}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {50, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealInput We_PLL annotation(
        Placement(visible = true, transformation(origin = {-58, 14}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    initial equation
      thetaR = 0;
    equation
// Relações angulares:
      der(thetaR) = Polos / 2 * Wrm;
      alpha = thetaE_PLL - thetaR;
// Equações de controle:
      Iqr = -(Lls + Lm) / Lm * (2 / 3 * (2 / Polos) * We_PLL * Te_esp / Vqs_PLL);
      Idr = (Vqs_PLL / We_PLL - (Lls + Lm) * (2 / 3 * Qs_esp / Vqs_PLL)) / Lm;
// Sinal para a fonte de corrente:
      {Iqr, Idr, 0} = Park1.qd0;
      iabcr[:] = Park1.abc;
      alpha = Park1.theta;
      annotation(
        Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-1, 0}, rotation = 180, lineColor = {0, 0, 255}, extent = {{-44, 37}, {44, -37}}, textString = "RSC"), Text(origin = {-90, -71}, lineColor = {0, 0, 255}, extent = {{-6, 15}, {6, -15}}, textString = "Wrm"), Text(origin = {-31, -154}, lineColor = {255, 255, 255}, extent = {{-11, 6}, {11, -6}}, textString = "Qesp"), Text(origin = {77, -81}, lineColor = {0, 0, 255}, extent = {{-17, 17}, {17, -17}}, textString = "thetaE_PLL"), Text(origin = {83, -39}, lineColor = {0, 0, 255}, extent = {{-13, 11}, {13, -11}}, textString = "Vqs_PLL"), Text(origin = {-51, -91}, lineColor = {0, 0, 255}, extent = {{-9, 7}, {9, -7}}, textString = "Te"), Text(origin = {50, -91}, lineColor = {0, 0, 255}, extent = {{-10, 7}, {10, -7}}, textString = "Qs"), Text(origin = {85, -59}, lineColor = {0, 0, 255}, extent = {{-11, 9}, {11, -9}}, textString = "We_PLL")}),
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
    end CONTROL_RSC;

    model CONTROL_GSC
      import SI = Modelica.Units.SI;
      import pi = Modelica.Constants.pi;
      CONTROL_model.tpark Park1;
      //  Parâmetros MIT 2MW
      parameter SI.Inductance Lls = 6.32e-5 "Stator leakage inductance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter SI.Inductance Lm = 1.8944e-3 "Magnetizing inductance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter Integer Polos = 4 "Number of poles" annotation(
        Dialog(group = "Mechanical Data"));
      Modelica.Blocks.Interfaces.RealOutput iabcg[3] annotation(
        Placement(visible = true, transformation(origin = {110, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealInput Vcc_esp annotation(
        Placement(visible = true, transformation(origin = {-50, -74}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {50, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealInput Idg_esp annotation(
        Placement(visible = true, transformation(origin = {-50, -46}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-50, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealInput theta_PLL annotation(
        Placement(visible = true, transformation(origin = {-120, 34}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
// Sinal para a fonte de corrente:
      {Vcc_esp, Idg_esp, 0} = Park1.qd0;
      iabcg = Park1.abc;
      theta_PLL = Park1.theta;
      annotation(
        Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-1, 0}, rotation = 180, lineColor = {0, 0, 255}, extent = {{-44, 37}, {44, -37}}, textString = "GSC"), Text(origin = {-31, -154}, lineColor = {255, 255, 255}, extent = {{-11, 6}, {11, -6}}, textString = "Qesp")}),
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
    end CONTROL_GSC;

    model tpark
      import Modelica.Constants;
      import Modelica.SIunits;
      //
      Modelica.Blocks.Interfaces.RealInput theta annotation(
        Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, 100}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
      Modelica.Blocks.Interfaces.RealOutput qd0[3] annotation(
        Placement(visible = true, transformation(origin = {120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput abc[3] annotation(
        Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    protected
      constant Real phi = 2 * Constants.pi / 3;
      constant Real k = 2 / 3;
    equation
      qd0[1] = k * (abc[1] * cos(theta) + abc[2] * cos(theta - phi) + abc[3] * cos(theta + phi));
      qd0[2] = k * (abc[1] * sin(theta) + abc[2] * sin(theta - phi) + abc[3] * sin(theta + phi));
      qd0[3] = k * (abc[1] + abc[2] + abc[3]) / 2;
      annotation(
        Icon(coordinateSystem(initialScale = 0.4, grid = {0.5, 0.5}), graphics = {Rectangle(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-100, 100}, {100, -100}}), Line(points = {{-100, -100}, {100, 100}, {100, 100}}, thickness = 0.5), Text(origin = {-30, 22}, extent = {{-30, 22}, {30, -22}}, textString = "abc"), Text(origin = {30, -22}, extent = {{-30, 22}, {30, -22}}, textString = "qd0")}),
        Diagram);
    end tpark;

    model PLL
      import SI = Modelica.Units.SI;
      import pi = Modelica.Constants.pi;
      parameter SI.Voltage Vm_ref = 690 * sqrt(2 / 3) "Voltage PLL" annotation(
        Dialog(group = "Control PLL Data"));
      parameter SI.AngularFrequency We_ref = 120 * Modelica.Constants.pi "Frequency PLL" annotation(
        Dialog(group = "Control PLL Data"));
      parameter Real zeta_PLL = 1 "Damping constant PLL" annotation(
        Dialog(group = "Control PLL Data"));
      parameter SI.AngularFrequency Wn_PLL = 377 "Natural frequency PLL" annotation(
        Dialog(group = "Control PLL Data"));
      parameter Real kiPll = Wn_PLL ^ 2 / Vm_ref "Ki of PLL" annotation(
        Dialog(group = "Control PLL Data"));
      parameter Real kpPll = 2 * zeta_PLL * Wn_PLL / Vm_ref "Kp of PLL" annotation(
        Dialog(group = "Control PLL Data"));
      Modelica.Electrical.Polyphase.Sensors.VoltageSensor voltageSensor annotation(
        Placement(visible = true, transformation(origin = {-66, 24}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Basic.Star star annotation(
        Placement(visible = true, transformation(origin = {-66, 6}, extent = {{-8, -8}, {8, 8}}, rotation = -90)));
      Modelica.Electrical.Analog.Basic.Ground ground annotation(
        Placement(visible = true, transformation(origin = {-66, -12}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Interfaces.PositivePlug vabc annotation(
        Placement(visible = true, transformation(origin = {-102, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      AERO_model.CONTROL_model.tpark tpark annotation(
        Placement(visible = true, transformation(origin = {-30, 24}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput VqPll annotation(
        Placement(visible = true, transformation(origin = {100, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput WePll annotation(
        Placement(visible = true, transformation(origin = {8, -62}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput ThetaPll annotation(
        Placement(visible = true, transformation(origin = {-30, -62}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Continuous.Integrator integrator(initType = Modelica.Blocks.Types.Init.InitialOutput, y(fixed = true), y_start = 0) annotation(
        Placement(visible = true, transformation(origin = {-8, -16}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Continuous.PI pi(T = kpPll / kiPll, initType = Modelica.Blocks.Types.Init.InitialOutput, k = -kpPll, y_start = We_ref) annotation(
        Placement(visible = true, transformation(origin = {62, -16}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    initial equation
//der(WePll) = 0;
    equation
      connect(star.plug_p, voltageSensor.plug_n) annotation(
        Line(points = {{-66, 14}, {-66, 18}}, color = {0, 0, 255}));
      connect(ground.p, star.pin_n) annotation(
        Line(points = {{-66, -6}, {-66, -2}}, color = {0, 0, 255}));
      connect(vabc, voltageSensor.plug_p) annotation(
        Line(points = {{-102, 30}, {-66, 30}}, color = {0, 0, 255}));
      connect(voltageSensor.v, tpark.abc) annotation(
        Line(points = {{-59, 24}, {-40, 24}}, color = {0, 0, 127}, thickness = 0.5));
      connect(tpark.qd0[1], VqPll) annotation(
        Line(points = {{-20, 24}, {100, 24}}, color = {0, 0, 127}));
      connect(integrator.y, tpark.theta) annotation(
        Line(points = {{-18, -16}, {-30, -16}, {-30, 14}}, color = {0, 0, 127}));
      connect(integrator.y, ThetaPll) annotation(
        Line(points = {{-18, -16}, {-30, -16}, {-30, -62}}, color = {0, 0, 127}));
      connect(tpark.qd0[2], pi.u) annotation(
        Line(points = {{-20, 24}, {82, 24}, {82, -16}, {74, -16}}, color = {0, 0, 127}));
      connect(pi.y, integrator.u) annotation(
        Line(points = {{52, -16}, {4, -16}}, color = {0, 0, 127}));
      connect(pi.y, WePll) annotation(
        Line(points = {{52, -16}, {8, -16}, {8, -62}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {1, -4}, lineColor = {0, 0, 255}, extent = {{-45, 32}, {45, -32}}, textString = "PLL"), Text(origin = {78, 62}, lineColor = {0, 0, 255}, extent = {{-18, 18}, {18, -18}}, textString = "VqPll"), Text(origin = {81, -1}, lineColor = {0, 0, 255}, extent = {{-19, 11}, {19, -11}}, textString = "WePll"), Text(origin = {76, -60}, lineColor = {0, 0, 255}, extent = {{-24, 20}, {24, -20}}, textString = "ThetaPll")}),
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
    end PLL;

    model PI_Astrom
      parameter Real kp = 1;
      parameter Real ki = 1;
      Modelica.Blocks.Interfaces.RealInput m annotation(
        Placement(visible = true, transformation(origin = {-100, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput y annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput r annotation(
        Placement(visible = true, transformation(origin = {-100, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-1.77636e-15, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Math.Add add(k2 = -1) annotation(
        Placement(visible = true, transformation(origin = {-36, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Continuous.Integrator integrator(initType = Modelica.Blocks.Types.Init.SteadyState, k = ki) annotation(
        Placement(visible = true, transformation(origin = {0, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Gain gain(k = kp) annotation(
        Placement(visible = true, transformation(origin = {-16, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Add add1(k2 = -1) annotation(
        Placement(visible = true, transformation(origin = {38, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(m, add.u2) annotation(
        Line(points = {{-100, -20}, {-48, -20}, {-48, 0}}, color = {0, 0, 127}));
      connect(r, add.u1) annotation(
        Line(points = {{-100, 20}, {-48, 20}, {-48, 12}}, color = {0, 0, 127}));
      connect(add.y, integrator.u) annotation(
        Line(points = {{-25, 6}, {-12, 6}}, color = {0, 0, 127}));
      connect(m, gain.u) annotation(
        Line(points = {{-100, -20}, {-28, -20}}, color = {0, 0, 127}));
      connect(gain.y, add1.u2) annotation(
        Line(points = {{-5, -20}, {26, -20}, {26, -6}}, color = {0, 0, 127}));
      connect(integrator.y, add1.u1) annotation(
        Line(points = {{12, 6}, {26, 6}}, color = {0, 0, 127}));
      connect(add1.y, y) annotation(
        Line(points = {{50, 0}, {110, 0}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Ellipse(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {0, 1}, lineColor = {0, 0, 255}, extent = {{-58, 29}, {58, -29}}, textString = "PI")}));
    end PI_Astrom;
  end CONTROL_model;

  package Examples
    extends Modelica.Icons.ExamplesPackage;

    model windturbineDFIG_teste1
      extends Modelica.Icons.Example;
      import pi = Modelica.Constants.pi;
      AERO_model.DFIG_model.MIT mit(D = 0) annotation(
        Placement(visible = true, transformation(origin = {-26, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor reactivePowerSensor annotation(
        Placement(visible = true, transformation(origin = {64, 48}, extent = {{-5, 5}, {5, -5}}, rotation = 180)));
      Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
        Placement(visible = true, transformation(origin = {76, -48}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Basic.Star star1 annotation(
        Placement(visible = true, transformation(origin = {76, -36}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Sources.CosineVoltage rede(V = fill(sqrt(2 / 3) * 690, 3), f = fill(60, 3)) annotation(
        Placement(visible = true, transformation(origin = {76, -22}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
      AERO_model.CONVERSOR_model.CONVERSOR conversor annotation(
        Placement(visible = true, transformation(origin = {17, 21}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
      AERO_model.CONTROL_model.CONTROL_MAQ control_maq(Ki_Qg = 10, Ki_Wrm = 1e6, Kp_Wrm = 1e6) annotation(
        Placement(visible = true, transformation(origin = {17, -9}, extent = {{-13, -13}, {13, 13}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation(
        Placement(visible = true, transformation(origin = {-50, 32}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
      Modelica.Blocks.Sources.Constant const(k = 0) annotation(
        Placement(visible = true, transformation(origin = {90, -14}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
      TURBINA_model.EOLICA eolica annotation(
        Placement(visible = true, transformation(origin = {-76, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      AERO_model.CONTROL_model.CRTL_TUR crtl_tur annotation(
        Placement(visible = true, transformation(origin = {-76, -14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Ramp ramp(duration = 0, height = -1, offset = 12, startTime = 5) annotation(
        Placement(visible = true, transformation(origin = {-106, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(reactivePowerSensor.plug_n, mit.plug_s) annotation(
        Line(points = {{59, 48}, {-25, 48}, {-25, 47}}, color = {0, 0, 255}));
      connect(ground1.p, star1.pin_n) annotation(
        Line(points = {{76, -44}, {76, -42}}, color = {0, 0, 255}));
      connect(rede.plug_n, star1.plug_p) annotation(
        Line(points = {{76, -28}, {76, -30}}, color = {0, 0, 255}));
      connect(reactivePowerSensor.plug_p, rede.plug_p) annotation(
        Line(points = {{69, 48}, {76, 48}, {76, -16}}, color = {0, 0, 255}));
      connect(conversor.PlugRSC, mit.plug_r) annotation(
        Line(points = {{0.5, 21}, {-22, 21}, {-22, 30}}, color = {0, 0, 255}));
      connect(conversor.PlugGSC, reactivePowerSensor.plug_n) annotation(
        Line(points = {{33.5, 21}, {59, 21}, {59, 48}}, color = {0, 0, 255}));
      connect(control_maq.iabcr, conversor.refRSC) annotation(
        Line(points = {{7.9, 0.1}, {7.9, 7.6}}, color = {0, 0, 127}, thickness = 0.5));
      connect(control_maq.iabcg, conversor.refGSC) annotation(
        Line(points = {{26.1, 0.1}, {26.1, 7.6}}, color = {0, 0, 127}, thickness = 0.5));
      connect(control_maq.Vsmed, rede.plug_p) annotation(
        Line(points = {{31.3, -15.5}, {54.3, -15.5}, {54.3, -16}, {76.3, -16}}, color = {0, 0, 255}));
      connect(control_maq.Qgmed, reactivePowerSensor.reactivePower) annotation(
        Line(points = {{31.3, -10.3}, {64.3, -10.3}, {64.3, 41.7}}, color = {0, 0, 127}));
      connect(mit.eixo, speedSensor.flange) annotation(
        Line(points = {{-37, 38}, {-50, 38}}));
      connect(speedSensor.w, control_maq.Wrmmed) annotation(
        Line(points = {{-50, 25}, {-50, -10.6}, {3, -10.6}}, color = {0, 0, 127}));
      connect(conversor.Vcc, control_maq.Vccmed) annotation(
        Line(points = {{17, 10.2}, {17, 0.2}}, color = {0, 0, 127}));
      connect(control_maq.Qgref, const.y) annotation(
        Line(points = {{31.3, -12.9}, {59.3, -12.9}, {59.3, -14}, {86, -14}}, color = {0, 0, 127}));
      connect(eolica.flange_Eixo, mit.eixo) annotation(
        Line(points = {{-64, 38}, {-36, 38}}));
      connect(eolica.beta, crtl_tur.beta) annotation(
        Line(points = {{-76, 28}, {-76, -6}}, color = {0, 0, 127}));
      connect(crtl_tur.Wrm_opt, control_maq.Wrmref) annotation(
        Line(points = {{-64, -14}, {2, -14}, {2, -12}}, color = {0, 0, 127}));
      connect(eolica.Vw, ramp.y) annotation(
        Line(points = {{-86, 38}, {-95, 38}}, color = {0, 0, 127}));
      connect(ramp.y, crtl_tur.Vw) annotation(
        Line(points = {{-95, 38}, {-94, 38}, {-94, -14}, {-88, -14}}, color = {0, 0, 127}));
    protected
      annotation(
        experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-06, Interval = 0.00150002));
    end windturbineDFIG_teste1;

    model teste
      AERO_model.DFIG_model.MIT mit_semderEstator annotation(
        Placement(visible = true, transformation(origin = {18, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Basic.Star star1 annotation(
        Placement(visible = true, transformation(origin = {56, 58}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
      Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
        Placement(visible = true, transformation(origin = {56, 46}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sources.CosineVoltage rede(V = fill(sqrt(2 / 3) * 690, 3), f = fill(60, 3)) annotation(
        Placement(visible = true, transformation(origin = {56, 72}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
      Modelica.Mechanics.Rotational.Sources.Torque torque annotation(
        Placement(visible = true, transformation(origin = {-20, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Ramp ramp(duration = 0, height = 10610.3295, offset = 0, startTime = 10) annotation(
        Placement(visible = true, transformation(origin = {-76, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(
        Placement(visible = true, transformation(origin = {56, -18}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Electrical.Polyphase.Sources.CosineVoltage cosineVoltage(V = fill(sqrt(2 / 3) * 690, 3), f = fill(60, 3)) annotation(
        Placement(visible = true, transformation(origin = {56, 8}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Sources.Torque torque1 annotation(
        Placement(visible = true, transformation(origin = {-20, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Polyphase.Basic.Star star annotation(
        Placement(visible = true, transformation(origin = {56, -6}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
  Modelica.Electrical.Analog.Basic.Ground ground2 annotation(
        Placement(visible = true, transformation(origin = {60, -90}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Electrical.Polyphase.Sources.CosineVoltage cosineVoltage1(V = fill(sqrt(2 / 3) * 690, 3), f = fill(60, 3)) annotation(
        Placement(visible = true, transformation(origin = {60, -64}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Sources.Torque torque2 annotation(
        Placement(visible = true, transformation(origin = {-16, -66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Polyphase.Basic.Star star2 annotation(
        Placement(visible = true, transformation(origin = {60, -78}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
  DFIG_model.MIT_complet mIT_complet annotation(
        Placement(visible = true, transformation(origin = {18, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  DFIG_model.MIT_semderRotor mIT_semderRotor annotation(
        Placement(visible = true, transformation(origin = {22, -66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(ground1.p, star1.pin_n) annotation(
        Line(points = {{56, 50}, {56, 52}}, color = {0, 0, 255}));
      connect(rede.plug_n, star1.plug_p) annotation(
        Line(points = {{56, 66}, {56, 64}}, color = {0, 0, 255}));
      connect(ramp.y, torque.tau) annotation(
        Line(points = {{-65, 70}, {-33, 70}}, color = {0, 0, 127}));
      connect(cosineVoltage.plug_n, star.plug_p) annotation(
        Line(points = {{56, 2}, {56, 0}}, color = {0, 0, 255}));
      connect(ground.p, star.pin_n) annotation(
        Line(points = {{56, -14}, {56, -12}}, color = {0, 0, 255}));
      connect(cosineVoltage1.plug_n, star2.plug_p) annotation(
        Line(points = {{60, -70}, {60, -72}}, color = {0, 0, 255}));
      connect(ground2.p, star2.pin_n) annotation(
        Line(points = {{60, -86}, {60, -84}}, color = {0, 0, 255}));
      connect(torque1.flange, mIT_complet.eixo) annotation(
        Line(points = {{-10, 6}, {8, 6}}));
      connect(mIT_complet.plug_s, cosineVoltage.plug_p) annotation(
        Line(points = {{20, 16}, {56, 16}, {56, 14}}, color = {0, 0, 255}));
      connect(mIT_complet.plug_r, star.plug_p) annotation(
        Line(points = {{22, -2}, {56, -2}, {56, 0}}, color = {0, 0, 255}));
      connect(mIT_semderRotor.plug_s, cosineVoltage1.plug_p) annotation(
        Line(points = {{24, -56}, {60, -56}, {60, -58}}, color = {0, 0, 255}));
      connect(mIT_semderRotor.plug_r, star2.plug_p) annotation(
        Line(points = {{26, -74}, {60, -74}, {60, -72}}, color = {0, 0, 255}));
      connect(mIT_semderRotor.eixo, torque2.flange) annotation(
        Line(points = {{12, -66}, {-6, -66}}));
  connect(ramp.y, torque1.tau) annotation(
        Line(points = {{-64, 70}, {-44, 70}, {-44, 6}, {-32, 6}}, color = {0, 0, 127}));
  connect(ramp.y, torque2.tau) annotation(
        Line(points = {{-64, 70}, {-44, 70}, {-44, -66}, {-28, -66}}, color = {0, 0, 127}));
  connect(torque.flange, mit_semderEstator.eixo) annotation(
        Line(points = {{-10, 70}, {8, 70}}));
  connect(mit_semderEstator.plug_s, rede.plug_p) annotation(
        Line(points = {{20, 80}, {56, 80}, {56, 78}}, color = {0, 0, 255}));
  connect(mit_semderEstator.plug_r, star1.plug_p) annotation(
        Line(points = {{22, 62}, {56, 62}, {56, 64}}, color = {0, 0, 255}));
      annotation(
        experiment(StartTime = 0, StopTime = 100, Tolerance = 1e-06, Interval = 0.001));
    end teste;
    
    model teste_curto
    import pi = Modelica.Constants.pi;
      Modelica.Blocks.Sources.Ramp ramp(duration = 0, height = 0, offset = 0.1 *10610.3295, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {-76, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Real mysignal[3];
    Modelica.Electrical.Polyphase.Sources.SignalVoltage signalVoltage1 annotation(
        Placement(visible = true, transformation(origin = {56, 10}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
    Modelica.Mechanics.Rotational.Sources.Torque torque1 annotation(
        Placement(visible = true, transformation(origin = {-20, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    DFIG_model.MIT_complet mIT_complet annotation(
        Placement(visible = true, transformation(origin = {18, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Electrical.Polyphase.Basic.Star star annotation(
        Placement(visible = true, transformation(origin = {56, -6}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
    Modelica.Electrical.Analog.Basic.Ground ground annotation(
        Placement(visible = true, transformation(origin = {56, -18}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    equation
      if time <= 0.11 then
        mysignal[:] = {sin(120 * pi * time), sin(120 * pi * time - 2 * pi / 3), sin(120 * pi * time + 2 * pi / 3)};
      elseif time > 0.1 and time <= 0.1 + 1 / 60 then
        mysignal[:] = {0, 0, 0};
      else
        mysignal[:] = {sin(120 * pi * time), sin(120 * pi * time - 2 * pi / 3), sin(120 * pi * time + 2 * pi / 3)};
      end if;
      signalVoltage.v[:] = mysignal[:];
      signalVoltage1.v[:] = mysignal[:];
      signalVoltage2.v[:] = mysignal[:];
      connect(ramp.y, torque1.tau) annotation(
        Line(points = {{-64, 70}, {-44, 70}, {-44, 6}, {-32, 6}}, color = {0, 0, 127}));
      connect(mIT_complet.plug_s, signalVoltage1.plug_p) annotation(
        Line(points = {{20, 16}, {56, 16}}, color = {0, 0, 255}));
      connect(torque1.flange, mIT_complet.eixo) annotation(
        Line(points = {{-10, 6}, {8, 6}}));
      connect(signalVoltage1.plug_n, star.plug_p) annotation(
        Line(points = {{56, 4}, {56, 0}}, color = {0, 0, 255}));
      connect(mIT_complet.plug_r, star.plug_p) annotation(
        Line(points = {{22, -2}, {56, -2}, {56, 0}}, color = {0, 0, 255}));
      connect(ground.p, star.pin_n) annotation(
        Line(points = {{56, -14}, {56, -12}}, color = {0, 0, 255}));
      annotation(
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.001));
    end teste_curto;
  end Examples;
  annotation(
    uses(Modelica(version = "4.0.0")));
end AERO_model;