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
      parameter Real Dtur = 0.3925 "Viscous turbine friction" annotation(
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
      cp = CP(Wtur/N, Vw, transferFunction.y);
      Ptur = 1 / 2 * (par * A * cp * Vw ^ 3);
      Ttur = Ptur / Wtur;
    // Equações mecânicas:
      Jtur * der(Wtur) = Ttur + Teixo - Dtur * Wtur;
      connect(beta, transferFunction.u) annotation(
        Line(points = {{-34, 80}, {8, 80}}, color = {0, 0, 127}));
      connect(transferFunction.y, slewRateLimiter.u) annotation(
        Line(points = {{32, 80}, {42, 80}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Ellipse(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{22, -56}, {22, -56}}), Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Rectangle(extent = {{-4, 4}, {4, -4}}), Polygon(origin = {-50, -28}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{50, 24}, {-50, -32}, {10, 30}, {46, 32}, {46, 30}, {50, 24}}), Polygon(origin = {42, -19}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-38, 23}, {58, -41}, {-26, -17}, {-42, 15}, {-42, 15}, {-38, 23}}), Polygon(origin = {3, 50}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-7, -46}, {-7, 50}, {15, -16}, {1, -46}, {-3, -46}, {-7, -46}}), Ellipse(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 0, extent = {{-6, 6}, {6, -6}})}),
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
    end TURBINA;

    model CRTL_TUR
      Modelica.Blocks.Interfaces.RealInput Vw annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-112, 1.77636e-15}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput beta annotation(
        Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
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
       
    
      Modelica.Blocks.Tables.CombiTable1Ds combiTable1Ds(extrapolation = Modelica.Blocks.Types.Extrapolation.LastTwoPoints, fileName = "/home/uemura/mypy/TCC/Simu2/mybeta.mat", tableName = "beta", tableOnFile = true, verboseRead = false) annotation(
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
        Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {4, -1}, lineColor = {0, 0, 255}, extent = {{-66, 45}, {66, -45}}, textString = "CTRL
TUR")}),
        experiment(StartTime = 0, StopTime = 510, Tolerance = 1e-06, Interval = 0.005));
    end CRTL_TUR;

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
        Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 60}, {100, -60}}), Text(origin = {-2, 0}, lineColor = {0, 0, 255}, extent = {{-62, 18}, {62, -18}}, textString = "%name")}));
    end EIXO;
  end TURBINA_model;

  package DFIG_model
    extends Modelica.Icons.Package;

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
      tpark Park1;
      tpark Park2;
      tpark Park3;
      tpark Park4;
      ElecInterface.PositivePlug plug_s annotation(
        Placement(visible = true, transformation(origin = {-28, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Interfaces.NegativePlug plug_r annotation(
        Placement(visible = true, transformation(origin = {10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {40, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      MechInterface.Flange_a eixo annotation(
        Placement(visible = true, transformation(origin = {-4, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    initial equation
      if RP then
        theta_e = 0;
        theta_rm = 0;
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
      J * der(Wrm) = +Tm + Te - D * Wrm;
      Wr = Polos / 2 * Wrm;
      annotation(
        experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-06, Interval = 0.0001),
        Diagram(coordinateSystem(extent = {{-20, 20}, {20, -20}})),
        Icon(graphics = {Bitmap(extent = {{20, 0}, {20, 0}}), Polygon(origin = {10, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-70, 60}, {-90, 40}, {-90, -40}, {-70, -60}, {70, -60}, {90, -40}, {90, 40}, {70, 60}, {-70, 60}, {-70, 60}}), Rectangle(origin = {-90, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, extent = {{-10, 8}, {10, -8}}), Rectangle(origin = {10, 70}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-30, 10}, {30, -10}}), Rectangle(origin = {40, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-24, 22}, {24, -22}}), Rectangle(origin = {40, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, extent = {{-24, 8}, {24, -8}}), Line(origin = {44.32, -37.76}, points = {{-2.19338, -22.7774}, {-2.19338, 23.2226}, {-2.19338, 23.2226}}, color = {0, 0, 255}), Line(origin = {51.13, -36.57}, points = {{5, 23}, {5, -15}, {5, -23}, {5, -23}, {5, -23}}, color = {0, 0, 255}), Line(origin = {33.23, -36.73}, points = {{-5, 23}, {-5, -15}, {-5, -23}, {-5, -23}, {-5, -23}}, color = {0, 0, 255}), Rectangle(origin = {42, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-4, 14}, {4, -14}}), Rectangle(origin = {56, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-4, 14}, {4, -14}}), Rectangle(origin = {28, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-4, 14}, {4, -14}}), Line(origin = {-48.82, 0.06}, points = {{-11.1784, 59.9389}, {-11.1784, -60.0611}, {-11.1784, -60.0611}}, color = {0, 0, 255}), Line(origin = {80, 0}, points = {{0, 60}, {0, -60}, {0, -60}}, color = {0, 0, 255}), Rectangle(origin = {40, -67}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-20, 7}, {20, -7}}), Line(origin = {10, 0}, points = {{-90, -40}, {-90, 40}, {-70, 60}, {70, 60}, {90, 40}, {90, -40}, {70, -60}, {-70, -60}, {-90, -40}, {-90, -40}}, color = {0, 0, 255}), Text(origin = {-24, 3}, lineColor = {0, 0, 255}, extent = {{-36, 19}, {36, -19}}, textString = "%name")}));
    end MIT;

    model CONTROL
      import SI = Modelica.Units.SI;
      import pi = Modelica.Constants.pi;
      SI.Current Iqr, Idr;
      SI.Angle thetaR, alpha;
      tpark Park1;
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
        Placement(visible = true, transformation(origin = {110, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {109, -1}, extent = {{-9, -9}, {9, 9}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Vqs_PLL annotation(
        Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput thetaE_PLL annotation(
        Placement(visible = true, transformation(origin = {-120, -52}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Te_esp annotation(
        Placement(visible = true, transformation(origin = {-50, -74}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-40, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealInput Qs_esp annotation(
        Placement(visible = true, transformation(origin = {30, -82}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {40, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealInput We_PLL annotation(
        Placement(visible = true, transformation(origin = {-58, 14}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
        Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {-1, 0}, rotation = 180, lineColor = {0, 0, 255}, extent = {{-44, 37}, {44, -37}}, textString = "CTRL
DFIG"), Text(origin = {-90, -71}, lineColor = {0, 0, 255}, extent = {{-6, 15}, {6, -15}}, textString = "Wrm"), Text(origin = {-31, -154}, lineColor = {255, 255, 255}, extent = {{-11, 6}, {11, -6}}, textString = "Qesp"), Text(origin = {-81, 1}, lineColor = {0, 0, 255}, extent = {{-17, 17}, {17, -17}}, textString = "thetaE_PLL"), Text(origin = {-85, 79}, lineColor = {0, 0, 255}, extent = {{-13, 11}, {13, -11}}, textString = "Vqs_PLL"), Text(origin = {-41, -89}, lineColor = {0, 0, 255}, extent = {{-9, 7}, {9, -7}}, textString = "Te"), Text(origin = {40, -89}, lineColor = {0, 0, 255}, extent = {{-10, 7}, {10, -7}}, textString = "Qs"), Text(origin = {-87, 41}, lineColor = {0, 0, 255}, extent = {{-11, 9}, {11, -9}}, textString = "We_PLL")}),
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
    end CONTROL;

    model PLL
      import SI = Modelica.Units.SI;
      import pi = Modelica.Constants.pi;
      parameter SI.Voltage Vm_ref = 690 * sqrt(2 / 3) "Voltage PLL" annotation(
        Dialog(group = "Control PLL Data"));
      parameter SI.AngularFrequency We_ref = 120 * Modelica.Constants.pi "Frequency PLL" annotation(
        Dialog(group = "Control PLL Data"));
      parameter Real zeta_PLL = 0.7 "Damping constant PLL" annotation(
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
      AERO_model.DFIG_model.tpark tpark annotation(
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

    model RV
      import SI = Modelica.Units.SI;
      parameter SI.MomentOfInertia Jtotal = 474.5721 + 59 "Total moment of inertia" annotation(
        Dialog(group = "Mechanical Data")); 
      parameter SI.RotationalDampingConstant Dtotal = 0.3925 "Viscous total friction" annotation(
        Dialog(group = "Mechanical Data"));
      parameter SI.Time ts_Wrm = 1 "Settiling time by speed" annotation(
        Dialog(group = "Velocity Control Data"));
      parameter Real zeta_Wrm = 0.7 "Damping constant by speed" annotation(
        Dialog(group = "Velocity Control Data"));
      parameter Real Md_Wrm = 10 "Max limiter SlewRate" annotation(
        Dialog(group = "Velocity Control Data"));
      parameter SI.Time Td_Wrm = 5 "Time constant SlewRate" annotation(
        Dialog(group = "Velocity Control Data"));
     
      parameter Real kp = 8 * Jtotal * zeta_Wrm ^ 2 / ts_Wrm - Dtotal;
      parameter Real ki = 16 * Jtotal * zeta_Wrm ^ 2 / ts_Wrm ^ 2;
      
      Modelica.Blocks.Interfaces.RealInput W annotation(
        Placement(visible = true, transformation(origin = {-92, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Te annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Wref annotation(
        Placement(visible = true, transformation(origin = {-92, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-1.77636e-15, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Math.Add add(k2 = -1) annotation(
        Placement(visible = true, transformation(origin = {-6, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Continuous.PI PI(T = kp / ki, initType = Modelica.Blocks.Types.Init.SteadyState, k = kp) annotation(
        Placement(visible = true, transformation(origin = {38, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(W, add.u2) annotation(
        Line(points = {{-92, -20}, {-28, -20}, {-28, -6}, {-18, -6}}, color = {0, 0, 127}));
      connect(add.y, PI.u) annotation(
        Line(points = {{6, 0}, {26, 0}}, color = {0, 0, 127}));
      connect(PI.y, Te) annotation(
        Line(points = {{49, 0}, {110, 0}}, color = {0, 0, 127}));
  connect(Wref, add.u1) annotation(
        Line(points = {{-92, 20}, {-28, 20}, {-28, 6}, {-18, 6}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Ellipse(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {0, 1}, lineColor = {0, 0, 255}, extent = {{-58, 29}, {58, -29}}, textString = "RV")}));
    end RV;

    model RQs
      parameter Real ki_Q = 30 "Integral constant by reactivepower" annotation(
        Dialog(group = "Reactive Power Control Data"));
      Modelica.Blocks.Interfaces.RealInput Qs annotation(
        Placement(visible = true, transformation(origin = {-68, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Qout annotation(
        Placement(visible = true, transformation(origin = {78, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Qsref annotation(
        Placement(visible = true, transformation(origin = {-68, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Math.Add add(k2 = -1) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Continuous.Integrator integrator(initType = Modelica.Blocks.Types.Init.SteadyState, k = ki_Q) annotation(
        Placement(visible = true, transformation(origin = {36, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(Qsref, add.u1) annotation(
        Line(points = {{-68, 20}, {-22, 20}, {-22, 6}, {-12, 6}}, color = {0, 0, 127}));
      connect(Qs, add.u2) annotation(
        Line(points = {{-68, -20}, {-22, -20}, {-22, -6}, {-12, -6}}, color = {0, 0, 127}));
      connect(integrator.y, Qout) annotation(
        Line(points = {{47, 0}, {78, 0}}, color = {0, 0, 127}));
      connect(add.y, integrator.u) annotation(
        Line(points = {{11, 0}, {24, 0}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Ellipse(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {0, 1}, lineColor = {0, 0, 255}, extent = {{-58, 29}, {58, -29}}, textString = "RQ")}));
    end RQs;
  end DFIG_model;

  package Examples
    extends Modelica.Icons.ExamplesPackage;

    model teste1
    extends Modelica.Icons.Example;
    equation

    end teste1;
  end Examples;
  annotation(
    uses(Modelica(version = "4.0.0")));
end AERO_model;