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
        Vw_rampa := Vmax_rampa*(1 - (tf_rampa - time)/(tf_rampa - ti_rampa));
      else
        Vw_rampa := 0;
      end if;
// Função rajada:
      if time >= ti_rajada and time < tf_rajada then
        Vw_rajada := Vmax_rajada/2*(1 - cos(2*pi*(time - ti_rajada)/(tf_rajada - ti_rajada)));
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
      input Real Lambda "Velocidade específica da turbina";
      input Real Beta "Ângulo de inclinação das pás";
      output Real Cp "Coeficiente de potência";
    protected
      Real Alpha;
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
      Alpha := 1/(1/(Lambda + Beta*C8) - C9/(Beta^3 + 1));
      Cp := C1*(C2/Alpha - C3*Beta - C4*Beta^C5 - C6)*exp(-C7/Alpha);
    end CP;

    model TURBINA
      // Inclusão de bibliotecas:
      import pi = Modelica.Constants.pi;
      import SI = Modelica.Units.SI;
      // Declaração de parâmetros:
      parameter Real Jtur = 474.5721 "Momento de inércia da turbina" annotation(
        Dialog(group = "Mechanical Data"));
      parameter Real Ktm = 6366.5064 "Elasticidade do eixo" annotation(
        Dialog(group = "Shaft Data"));
      parameter Real Dtm = 84.4577 "Atrito viscoso do eio" annotation(
        Dialog(group = "Shaft Data"));
      parameter Real par = 1.225 "Densidade do ar" annotation(
        Dialog(group = "Aerodynamic Data"));
      parameter Real R = 37.5 "Raio das pas" annotation(
        Dialog(group = "Aerodynamic Data"));
      parameter Real A = pi*R^2 "Area varrida pelas pas" annotation(
        Dialog(group = "Aerodynamic Data"));
      parameter Real N = 111.5 "Relação de engrenagens" annotation(
        Dialog(group = "Aerodynamic Data"));
      // Servo mecanísmo para CONTROL_maqe de passo:
      parameter Real kb = 0.7143 "kb do servo mecanismo" annotation(
        Dialog(group = "CONTROL_maq Data"));
      parameter Real tb = 1.1905 "ts do servo mecanismo" annotation(
        Dialog(group = "CONTROL_maq Data"));
      // Declaração de variáveis auxiliares:
      SI.AngularFrequency Wtur;
      SI.Torque Ttur;
      SI.Power Ptur;
      Real cp;
      // Declaração de recursos auxiliares:
      Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_Eixo annotation(
        Placement(visible = true, transformation(origin = {88, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Vw annotation(
        Placement(visible = true, transformation(origin = {-63, -7}, extent = {{-11, -11}, {11, 11}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput beta annotation(
        Placement(visible = true, transformation(origin = {-56, 62}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Mechanics.Rotational.Components.SpringDamper springDamper(c = Ktm, d = Dtm) annotation(
        Placement(visible = true, transformation(origin = {60, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.Torque torque annotation(
        Placement(visible = true, transformation(origin = {0, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J = Jtur) annotation(
        Placement(visible = true, transformation(origin = {30, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Continuous.TransferFunction transferFunction(a = {tb, 1}, b = {1}, initType = Modelica.Blocks.Types.Init.SteadyState) annotation(
        Placement(visible = true, transformation(origin = {10, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Nonlinear.Limiter limiter(uMax = 10) annotation(
        Placement(visible = true, transformation(origin = {38, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Continuous.Integrator tfBeta(initType = Modelica.Blocks.Types.Init.SteadyState, k = kb) annotation(
        Placement(visible = true, transformation(origin = {66, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Add add(k2 = -1) annotation(
        Placement(visible = true, transformation(origin = {-18, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    initial equation
      der(springDamper.flange_a.phi - springDamper.flange_b.phi) = 0;
      der(Wtur) = 0;
    equation
// Conexões com o flange:
      Ttur = torque.tau;
      Wtur = inertia.w;
// Convertendo velocidade em conjugado:
      cp = CP(R*Wtur/N/Vw, tfBeta.y);
      Ptur = 1/2*(par*A*cp*Vw^3);
      Ttur = Ptur/Wtur;
      connect(torque.flange, inertia.flange_a) annotation(
        Line(points = {{10, -2}, {20, -2}}));
      connect(inertia.flange_b, springDamper.flange_a) annotation(
        Line(points = {{40, -2}, {50, -2}}));
      connect(springDamper.flange_b, flange_Eixo) annotation(
        Line(points = {{70, -2}, {88, -2}}));
      connect(beta, add.u1) annotation(
        Line(points = {{-56, 62}, {-30, 62}}, color = {0, 0, 127}));
      connect(add.y, transferFunction.u) annotation(
        Line(points = {{-7, 56}, {-3, 56}}, color = {0, 0, 127}));
      connect(transferFunction.y, limiter.u) annotation(
        Line(points = {{21, 56}, {25, 56}}, color = {0, 0, 127}));
      connect(limiter.y, tfBeta.u) annotation(
        Line(points = {{49, 56}, {53, 56}}, color = {0, 0, 127}));
      connect(add.u2, tfBeta.y) annotation(
        Line(points = {{-30, 50}, {-32, 50}, {-32, 36}, {78, 36}, {78, 56}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Ellipse(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{22, -56}, {22, -56}}), Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Rectangle(extent = {{-4, 4}, {4, -4}}), Polygon(origin = {-50, -28}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{50, 24}, {-50, -32}, {10, 30}, {46, 32}, {46, 30}, {50, 24}}), Polygon(origin = {42, -19}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-38, 23}, {58, -41}, {-26, -17}, {-42, 15}, {-42, 15}, {-38, 23}}), Polygon(origin = {3, 50}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-7, -46}, {-7, 50}, {15, -16}, {1, -46}, {-3, -46}, {-7, -46}}), Ellipse(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 0, extent = {{-6, 6}, {6, -6}})}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
        Diagram(graphics = {Text(origin = {15, 9}, textColor = {0, 0, 255}, extent = {{-19, 1}, {19, -1}}, textString = "", textStyle = {TextStyle.Bold, TextStyle.Italic}), Rectangle(origin = {11, 86}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-77, 4}, {77, -4}}), Text(origin = {11, 87}, textColor = {0, 0, 255}, extent = {{-77, 3}, {77, -3}}, textString = "Malha do Servomecanísmo", textStyle = {TextStyle.Bold, TextStyle.Italic}), Rectangle(origin = {36, -8}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, extent = {{-64, 27}, {64, -27}}), Rectangle(origin = {36, 22}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-64, 4}, {64, -4}}), Rectangle(origin = {11, 56}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, extent = {{-77, 27}, {77, -27}}), Rectangle(origin = {-59, -8}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, extent = {{-21, 27}, {21, -27}}), Rectangle(origin = {-59, 22}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-21, 4}, {21, -4}}), Text(origin = {-59, 22}, textColor = {0, 0, 255}, extent = {{-21, 4}, {21, -4}}, textString = "Pino Vw", textStyle = {TextStyle.Bold, TextStyle.Italic}), Text(origin = {36, 22}, textColor = {0, 0, 255}, extent = {{-64, 4}, {64, -4}}, textString = "Conexões Mecânicas", textStyle = {TextStyle.Bold, TextStyle.Italic})}));
    end TURBINA;

    model CONTROL_tur
      Modelica.Blocks.Interfaces.RealInput Vw annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-112, 1.77636e-15}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput beta annotation(
        Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Interfaces.RealOutput Wrm_opt annotation(
        Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter Real R = 37.5 "Raio das pas" annotation(
        Dialog(group = "Turbine Data"));
      parameter Real N = 111.5 "Relação de engrenagens" annotation(
        Dialog(group = "Turbine Data"));
      parameter Real LBD_opt = 6.3279 "Velocidade específica ótima" annotation(
        Dialog(group = "Turbine Data"));
      parameter Real Vw_min = 4 "Velocidade mínima do vento" annotation(
        Dialog(group = "Turbine Data"));
      parameter Real Vw_nom = 12 "Velocidade nominal do vento" annotation(
        Dialog(group = "Turbine Data"));
      parameter Real Vw_max = 25 "Velocidade máxima do vento" annotation(
        Dialog(group = "Turbine Data"));
      parameter Real Vw_wmin = 8.071353643090237 "Velocidade do vento inicial no MPPT" annotation(
        Dialog(group = "Turbine Data"));
      parameter Real Vw_wmax = 11.175720428894175 "Velocidade do vento final no MPPT" annotation(
        Dialog(group = "Turbine Data"));
      Modelica.Blocks.Tables.CombiTable1Ds combiTable1Ds(extrapolation = Modelica.Blocks.Types.Extrapolation.LastTwoPoints, fileName = "/home/uemura/TCCJoseVictor/Simu/TCC/Simu2/mybeta.mat", tableName = "beta", tableOnFile = true, verboseRead = false) annotation(
        Placement(visible = true, transformation(origin = {50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    algorithm
      if Vw >= Vw_min and Vw < Vw_wmin then
        Wrm_opt := N*LBD_opt*Vw_wmin/R;
        beta := 0;
      elseif Vw >= Vw_wmin and Vw < Vw_wmax then
        Wrm_opt := N*LBD_opt*Vw/R;
        beta := 0;
      elseif Vw >= Vw_wmax and Vw < Vw_nom then
        Wrm_opt := N*LBD_opt*Vw_wmax/R;
        beta := 0;
      elseif Vw >= Vw_nom and Vw <= Vw_max then
        Wrm_opt := N*LBD_opt*Vw_wmax/R;
        combiTable1Ds.u := Vw - 12;
        beta := combiTable1Ds.y[1];
      end if;
      annotation(
        Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {7, -1}, textColor = {0, 0, 255}, extent = {{-73, 45}, {73, -45}}, textString = "REGIÕES")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
        experiment(StartTime = 0, StopTime = 510, Tolerance = 1e-06, Interval = 0.005));
    end CONTROL_tur;
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
      constant Real phi = 2*Constants.pi/3;
      constant Real k = 2/3;
    equation
      qd0[1] = k*(abc[1]*cos(theta) + abc[2]*cos(theta - phi) + abc[3]*cos(theta + phi));
      qd0[2] = k*(abc[1]*sin(theta) + abc[2]*sin(theta - phi) + abc[3]*sin(theta + phi));
      qd0[3] = k*(abc[1] + abc[2] + abc[3])/2;
      annotation(
        Icon(coordinateSystem(initialScale = 0.4, grid = {0.5, 0.5}, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Line(origin = {0, -1}, points = {{-100, -99}, {100, 99}, {100, 99}}), Text(origin = {-30, 61}, textColor = {0, 0, 255}, extent = {{-70, 19}, {70, -19}}, textString = "ABC"), Text(origin = {30, -62}, textColor = {0, 0, 255}, extent = {{-70, 20}, {70, -20}}, textString = "QD0")}),
        Diagram);
    end tpark;

    model MIT
      // Inclusão de bibliotecas:
      import SI = Modelica.Units.SI;
      import pi = Modelica.Constants.pi;
      import MechInterface = Modelica.Mechanics.Rotational.Interfaces;
      import ElecInterface = Modelica.Electrical.Polyphase.Interfaces;
      // Declaração de variáveis auxiliares:
      SI.Angle alpha, theta_r, theta_e, theta_rm;
      SI.AngularFrequency Wr, Wrm;
      SI.Voltage Vas, Vbs, Vcs, Var, Vbr, Vcr, Vqs, Vds, V0s, Vqr, Vdr, V0r;
      SI.Current Iqs, Ids, I0s, Iqr, Idr, I0r, Iar, Ibr, Icr, Ias, Ibs, Ics;
      SI.MagneticFlux fqs, fds, f0s, fqr, fdr, f0r;
      SI.Torque Te;
      Real s "%";
      // Declaração de parâmetros:
      parameter SI.AngularFrequency We = 120*pi;
      parameter SI.Resistance rs = 2.381e-3 "Resistência do estator" annotation(
        Dialog(group = "Dados elétricos"));
      parameter SI.Resistance rr = 2.381e-3 "Resistência do rotor" annotation(
        Dialog(group = "Dados elétricos"));
      parameter SI.Inductance Lls = 6.32e-5 "Indutância de dispersão do estator" annotation(
        Dialog(group = "Dados elétricos"));
      parameter SI.Inductance Llr = 5.04e-5 "Indutância de dispersão do rotor" annotation(
        Dialog(group = "Dados elétricos"));
      parameter SI.Inductance Lm = 1.8944e-3 "Indutância de magnetização" annotation(
        Dialog(group = "Dados elétricos"));
      parameter Integer Polos = 4 "Número de polos" annotation(
        Dialog(group = "Dados mecânicos"));
      parameter SI.MomentOfInertia J = 59 "Momento de inercia" annotation(
        Dialog(group = "Dados mecânicos"));
      parameter SI.RotationalDampingConstant D = 0 "Atrito viscoso" annotation(
        Dialog(group = "Dados mecânicos"));
      // Declaração de modelos auxiliares:
      tpark Park1;
      tpark Park2;
      tpark Park3;
      tpark Park4;
      Modelica.Electrical.Polyphase.Interfaces.PositivePlug plug_s annotation(
        Placement(visible = true, transformation(origin = {-4, 10}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {10, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Interfaces.NegativePlug plug_r annotation(
        Placement(visible = true, transformation(origin = {4, 10}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {40, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Interfaces.Flange_a eixo annotation(
        Placement(visible = true, transformation(origin = {-10, -8}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J = J) annotation(
        Placement(visible = true, transformation(origin = {-2.22045e-16, -8}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.Torque torque annotation(
        Placement(visible = true, transformation(origin = {10, -8}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
    initial equation
      theta_e = 0;
      theta_rm = 0;
      der(Wrm) = 0;
      der(fqs) = 0;
      der(fds) = 0;
      der(f0s) = 0;
      der(fqr) = 0;
      der(fdr) = 0;
      der(f0r) = 0;
    equation
//  Conexão dos plugs com as variaveis internas:
      {Vas, Vbs, Vcs} = plug_s.pin[:].v;
      {Ias, Ibs, Ics} = plug_s.pin[:].i;
      {Var, Vbr, Vcr} = plug_r.pin[:].v;
      {Iar, Ibr, Icr} = plug_r.pin[:].i;
//    Conexões mecânicas, com flange:
      theta_rm = inertia.phi;
      Te = torque.tau;
//  Relações angulares:
      der(theta_e) = We;
      der(theta_rm) = Wrm;
      theta_r = Polos/2*theta_rm;
      s = (We - Wr)/We;
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
//  Equações de fluxo e tensão do estator:
      fqs = Lls*Iqs + Lm*(Iqs + Iqr);
      fds = Lls*Ids + Lm*(Ids + Idr);
      f0s = Lls*I0s;
      Vqs = rs*Iqs + We*fds + der(fqs);
      Vds = rs*Ids - We*fqs + der(fds);
      V0s = rs*I0s + der(f0s);
//  Equações de fluxo e tensão do rotor:
      fqr = Llr*Iqr + Lm*(Iqs + Iqr);
      fdr = Llr*Idr + Lm*(Ids + Idr);
      f0r = Llr*I0r;
      Vqr = rr*Iqr + (We - Wr)*fdr + der(fqr);
      Vdr = rr*Idr - (We - Wr)*fqr + der(fdr);
      V0r = rr*I0r + der(f0r);
//  Modelo eletromecânico:
      Te = 3*Polos/4*(fds*Iqs - fqs*Ids);
      Wr = Polos/2*Wrm;
      connect(torque.flange, inertia.flange_b) annotation(
        Line(points = {{6, -8}, {4, -8}}));
      connect(inertia.flange_a, eixo) annotation(
        Line(points = {{-4, -8}, {-10, -8}}));
      annotation(
        experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-06, Interval = 0.0001),
        Diagram(coordinateSystem(extent = {{-20, 20}, {20, -20}}), graphics = {Rectangle(origin = {1, -8}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, extent = {{-19, 6}, {19, -6}}), Rectangle(origin = {1, -1}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-19, 1}, {19, -1}}), Rectangle(origin = {0, 7}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, extent = {{-10, 5}, {10, -5}}), Rectangle(origin = {0, 13}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-10, 1}, {10, -1}}), Text(origin = {1, -1}, textColor = {0, 0, 255}, extent = {{-19, 1}, {19, -1}}, textString = "Conexões Mecânicas", textStyle = {TextStyle.Bold, TextStyle.Italic}), Text(origin = {0, 13}, textColor = {0, 0, 255}, extent = {{-10, 1}, {10, -1}}, textString = "Conectores Elétricos", textStyle = {TextStyle.Bold, TextStyle.Italic})}),
        Icon(graphics = {Bitmap(extent = {{20, 0}, {20, 0}}), Polygon(origin = {10, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-70, 60}, {-90, 40}, {-90, -40}, {-70, -60}, {70, -60}, {90, -40}, {90, 40}, {70, 60}, {-70, 60}, {-70, 60}}), Rectangle(origin = {-90, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-10, 8}, {10, -8}}), Rectangle(origin = {10, 70}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-30, 10}, {30, -10}}), Rectangle(origin = {40, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-24, 22}, {24, -22}}), Rectangle(origin = {40, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, extent = {{-24, 8}, {24, -8}}), Line(origin = {44.32, -37.76}, points = {{-2.19338, -22.7774}, {-2.19338, 23.2226}, {-2.19338, 23.2226}}, color = {0, 0, 255}), Line(origin = {51.13, -36.57}, points = {{5, 23}, {5, -15}, {5, -23}, {5, -23}, {5, -23}}, color = {0, 0, 255}), Line(origin = {33.23, -36.73}, points = {{-5, 23}, {-5, -15}, {-5, -23}, {-5, -23}, {-5, -23}}, color = {0, 0, 255}), Rectangle(origin = {42, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-4, 14}, {4, -14}}), Rectangle(origin = {56, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-4, 14}, {4, -14}}), Rectangle(origin = {28, 0}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-4, 14}, {4, -14}}), Line(origin = {-48.82, 0.06}, points = {{-11.1784, 59.9389}, {-11.1784, -60.0611}, {-11.1784, -60.0611}}, color = {0, 0, 255}), Line(origin = {80, 0}, points = {{0, 60}, {0, -60}, {0, -60}}, color = {0, 0, 255}), Rectangle(origin = {40, -67}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-20, 7}, {20, -7}}), Line(origin = {10, 0}, points = {{-90, -40}, {-90, 40}, {-70, 60}, {70, 60}, {90, 40}, {90, -40}, {70, -60}, {-70, -60}, {-90, -40}, {-90, -40}}, color = {0, 0, 255}), Text(origin = {-24, 3}, textColor = {0, 0, 255}, extent = {{-36, 19}, {36, -19}}, textString = "%name")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
    end MIT;

    model CONTROL_maq
      // Bibliotecas necessárias:
      import SI = Modelica.Units.SI;
      import pi = Modelica.Constants.pi;
      // Criação de parâmetros:
      parameter SI.Inductance Lls = 6.32e-5 "Stator leakage inductance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter SI.Inductance Lm = 1.8944e-3 "Magnetizing inductance" annotation(
        Dialog(group = "Eletrical Data"));
      parameter Integer Polos = 4 "Number of poles" annotation(
        Dialog(group = "Mechanical Data"));
      // Componentes auxiliares:
      SI.Current Iqr, Idr;
      SI.Angle thetaR, alpha;
      tpark Park1;
      Modelica.Blocks.Interfaces.RealInput Wrm annotation(
        Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput iabcr[3] annotation(
        Placement(visible = true, transformation(origin = {120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {109, -1}, extent = {{-9, -9}, {9, 9}}, rotation = 0)));
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
      der(thetaR) = Polos/2*Wrm;
      alpha = thetaE_PLL - thetaR;
// Equações de CONTROL_maqe:
      Iqr = -(Lls + Lm)/Lm*(2/3*(2/Polos)*We_PLL*Te_esp/Vqs_PLL);
      Idr = (Vqs_PLL/We_PLL - (Lls + Lm)*(2/3*Qs_esp/Vqs_PLL))/Lm;
// Sinal para a fonte de corrente:
      {Iqr, Idr, 0} = Park1.qd0;
      iabcr[:] = Park1.abc;
      alpha = Park1.theta;
      annotation(
        Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(rotation = 180, textColor = {0, 0, 255}, extent = {{-81, 39}, {81, -39}}, textString = "EQ CORRENTES"), Text(origin = {-31, -154}, textColor = {255, 255, 255}, extent = {{-11, 6}, {11, -6}}, textString = "Qesp")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002));
    end CONTROL_maq;

    model PLL
      import SI = Modelica.Units.SI;
      import pi = Modelica.Constants.pi;
      parameter SI.Voltage Vm_ref = 690*sqrt(2/3) "Tensão do estator" annotation(
        Dialog(group = "CONTROL_maq PLL Data"));
      parameter SI.AngularFrequency We_ref = 120*Modelica.Constants.pi "Frequência do estator" annotation(
        Dialog(group = "CONTROL_maq PLL Data"));
      parameter Real zeta_PLL = 0.7 "Contante de amortecimento do PLL" annotation(
        Dialog(group = "CONTROL_maq PLL Data"));
      parameter SI.AngularFrequency Wn_PLL = 377 "Frequência natural do PLL" annotation(
        Dialog(group = "CONTROL_maq PLL Data"));
      parameter Real kiPll = Wn_PLL^2/Vm_ref "Ki do PLL" annotation(
        Dialog(group = "CONTROL_maq PLL Data"));
      parameter Real kpPll = 2*zeta_PLL*Wn_PLL/Vm_ref "Kp do PLL" annotation(
        Dialog(group = "CONTROL_maq PLL Data"));
      Modelica.Electrical.Polyphase.Sensors.VoltageSensor voltageSensor annotation(
        Placement(visible = true, transformation(origin = {-66, 30}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Basic.Star star annotation(
        Placement(visible = true, transformation(origin = {-66, 12}, extent = {{-8, -8}, {8, 8}}, rotation = -90)));
      Modelica.Electrical.Analog.Basic.Ground ground annotation(
        Placement(visible = true, transformation(origin = {-66, -6}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Interfaces.PositivePlug vabc annotation(
        Placement(visible = true, transformation(origin = {-102, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      AERO_model.DFIG_model.tpark tpark annotation(
        Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput VqPll annotation(
        Placement(visible = true, transformation(origin = {90, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput WePll annotation(
        Placement(visible = true, transformation(origin = {90, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput ThetaPll annotation(
        Placement(visible = true, transformation(origin = {90, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Continuous.Integrator integrator(initType = Modelica.Blocks.Types.Init.InitialOutput, y(fixed = true), y_start = 0) annotation(
        Placement(visible = true, transformation(origin = {10, 10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Blocks.Continuous.PI pi(T = kpPll/kiPll, initType = Modelica.Blocks.Types.Init.InitialOutput, k = -kpPll, y_start = We_ref) annotation(
        Placement(visible = true, transformation(origin = {50, 10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    equation
      connect(star.plug_p, voltageSensor.plug_n) annotation(
        Line(points = {{-66, 20}, {-66, 24}}, color = {0, 0, 255}));
      connect(ground.p, star.pin_n) annotation(
        Line(points = {{-66, 0}, {-66, 4}}, color = {0, 0, 255}));
      connect(vabc, voltageSensor.plug_p) annotation(
        Line(points = {{-102, 36}, {-66, 36}}, color = {0, 0, 255}));
      connect(voltageSensor.v, tpark.abc) annotation(
        Line(points = {{-59, 30}, {-40, 30}}, color = {0, 0, 127}, thickness = 0.5));
      connect(tpark.qd0[1], VqPll) annotation(
        Line(points = {{-20, 30}, {90, 30}}, color = {0, 0, 127}));
      connect(pi.y, integrator.u) annotation(
        Line(points = {{39, 10}, {22, 10}}, color = {0, 0, 127}));
      connect(integrator.y, tpark.theta) annotation(
        Line(points = {{0, 10}, {-30, 10}, {-30, 20}}, color = {0, 0, 127}));
      connect(pi.y, WePll) annotation(
        Line(points = {{40, 10}, {32, 10}, {32, -12}, {90, -12}}, color = {0, 0, 127}));
      connect(ThetaPll, integrator.y) annotation(
        Line(points = {{90, -32}, {-12, -32}, {-12, 10}, {0, 10}}, color = {0, 0, 127}));
      connect(tpark.qd0[2], pi.u) annotation(
        Line(points = {{-20, 30}, {72, 30}, {72, 10}, {62, 10}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {1, -4}, textColor = {0, 0, 255}, extent = {{-45, 32}, {45, -32}}, textString = "PLL")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
        Diagram);
    end PLL;

    model RV
      //
      import SI = Modelica.Units.SI;
      //
      parameter SI.MomentOfInertia Jtotal = 530 "Momento de inércia" annotation(
        Dialog(group = "Mechanical Data"));
      parameter SI.RotationalDampingConstant Dtotal = 0 "Atrito viscoso" annotation(
        Dialog(group = "Mechanical Data"));
      parameter SI.Time ts_Wrm = 5 "Tempo de acomodação para a velocidade" annotation(
        Dialog(group = "Velocity CONTROL_maq Data"));
      parameter Real zeta_Wrm = 0.7 "Coeficiente de amortecimento para a velocidade" annotation(
        Dialog(group = "Velocity CONTROL_maq Data"));
      //
      parameter Real kp = 8*Jtotal*zeta_Wrm^2/ts_Wrm - Dtotal;
      parameter Real ki = 16*Jtotal*zeta_Wrm^2/ts_Wrm^2;
      //
      Modelica.Blocks.Interfaces.RealInput W annotation(
        Placement(visible = true, transformation(origin = {-66, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Te annotation(
        Placement(visible = true, transformation(origin = {110, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Wref annotation(
        Placement(visible = true, transformation(origin = {-68, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Math.Add add(k2 = -1) annotation(
        Placement(visible = true, transformation(origin = {-10, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Continuous.Integrator integrator(initType = Modelica.Blocks.Types.Init.SteadyState, k = ki) annotation(
        Placement(visible = true, transformation(origin = {30, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Gain gain(k = kp) annotation(
        Placement(visible = true, transformation(origin = {10, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Add add1(k2 = -1) annotation(
        Placement(visible = true, transformation(origin = {70, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(W, add.u2) annotation(
        Line(points = {{-66, -20}, {-22, -20}, {-22, 8}}, color = {0, 0, 127}));
      connect(Wref, add.u1) annotation(
        Line(points = {{-68, 20}, {-22, 20}}, color = {0, 0, 127}));
      connect(add.y, integrator.u) annotation(
        Line(points = {{1, 14}, {18, 14}}, color = {0, 0, 127}));
      connect(integrator.y, add1.u1) annotation(
        Line(points = {{41, 14}, {58, 14}}, color = {0, 0, 127}));
      connect(gain.u, W) annotation(
        Line(points = {{-2, -20}, {-66, -20}}, color = {0, 0, 127}));
      connect(add1.y, Te) annotation(
        Line(points = {{81, 8}, {110, 8}}, color = {0, 0, 127}));
      connect(gain.y, add1.u2) annotation(
        Line(points = {{21, -20}, {58, -20}, {58, 2}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Ellipse(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {0, 1}, textColor = {0, 0, 255}, extent = {{-58, 29}, {58, -29}}, textString = "RV")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
        Diagram);
    end RV;

    model RQs
      parameter Real ki_Q = 30 "Integral constant by reactivepower" annotation(
        Dialog(group = "Reactive Power CONTROL_maq Data"));
      Modelica.Blocks.Interfaces.RealInput Qs annotation(
        Placement(visible = true, transformation(origin = {-60, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Qout annotation(
        Placement(visible = true, transformation(origin = {70, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Qsref annotation(
        Placement(visible = true, transformation(origin = {-60, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Math.Add add(k2 = -1) annotation(
        Placement(visible = true, transformation(origin = {-8, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Continuous.Integrator integrator(initType = Modelica.Blocks.Types.Init.SteadyState, k = ki_Q) annotation(
        Placement(visible = true, transformation(origin = {30, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(Qsref, add.u1) annotation(
        Line(points = {{-60, 20}, {-20, 20}}, color = {0, 0, 127}));
      connect(Qs, add.u2) annotation(
        Line(points = {{-60, -20}, {-20, -20}, {-20, 8}}, color = {0, 0, 127}));
      connect(integrator.y, Qout) annotation(
        Line(points = {{41, 14}, {70, 14}}, color = {0, 0, 127}));
      connect(add.y, integrator.u) annotation(
        Line(points = {{3, 14}, {18, 14}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Ellipse(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {0, 1}, textColor = {0, 0, 255}, extent = {{-58, 29}, {58, -29}}, textString = "RQ")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})));
    end RQs;

    model RV_comZeros
      //
      import SI = Modelica.Units.SI;
      //
      parameter SI.MomentOfInertia Jtotal = 59 "Momento de inércia" annotation(
        Dialog(group = "Mechanical Data"));
      parameter SI.RotationalDampingConstant Dtotal = 0 "Atrito viscoso" annotation(
        Dialog(group = "Mechanical Data"));
      parameter SI.Time ts_Wrm = 5 "Tempo de acomodação para a velocidade" annotation(
        Dialog(group = "Velocity CONTROL_maq Data"));
      parameter Real zeta_Wrm = 1 "Coeficiente de amortecimento para a velocidade" annotation(
        Dialog(group = "Velocity CONTROL_maq Data"));
      //
      parameter Real kp = 8*Jtotal*zeta_Wrm^2/ts_Wrm - Dtotal;
      parameter Real ki = 16*Jtotal*zeta_Wrm^2/ts_Wrm^2;
      //
      Modelica.Blocks.Interfaces.RealInput W annotation(
        Placement(visible = true, transformation(origin = {-66, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealOutput Te annotation(
        Placement(visible = true, transformation(origin = {110, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput Wref annotation(
        Placement(visible = true, transformation(origin = {-68, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Blocks.Math.Add add(k2 = -1) annotation(
        Placement(visible = true, transformation(origin = {-10, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Continuous.PI pi(T = kp/ki, initType = Modelica.Blocks.Types.Init.SteadyState, k = kp) annotation(
        Placement(visible = true, transformation(origin = {32, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(W, add.u2) annotation(
        Line(points = {{-66, -20}, {-22, -20}, {-22, 8}}, color = {0, 0, 127}));
      connect(Wref, add.u1) annotation(
        Line(points = {{-68, 20}, {-22, 20}}, color = {0, 0, 127}));
      connect(add.y, pi.u) annotation(
        Line(points = {{2, 14}, {20, 14}}, color = {0, 0, 127}));
      connect(pi.y, Te) annotation(
        Line(points = {{44, 14}, {110, 14}, {110, 8}}, color = {0, 0, 127}));
      annotation(
        Icon(graphics = {Ellipse(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text(origin = {0, 1}, textColor = {0, 0, 255}, extent = {{-58, 29}, {58, -29}}, textString = "RV")}, coordinateSystem(extent = {{-100, -100}, {100, 100}})),
        Diagram);
    end RV_comZeros;
  end DFIG_model;

  package Examples
    extends Modelica.Icons.ExamplesPackage;

    model windturbineDFIG_simuR123
      extends Modelica.Icons.Example;
      import pi = Modelica.Constants.pi;
      AERO_model.TURBINA_model.TURBINA turbina(ts = 1) annotation(
        Placement(visible = true, transformation(origin = {-12, 38}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
      DFIG_model.CONTROL_maq CONTROL_maq annotation(
        Placement(visible = true, transformation(origin = {32, -14}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
      Modelica.Blocks.Sources.Step Qref(height = 0, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {58, 2}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
      DFIG_model.PLL pll annotation(
        Placement(visible = true, transformation(origin = {0, -18}, extent = {{-6, 6}, {6, -6}}, rotation = 0)));
      DFIG_model.RQs rQs(ki_Q = 30) annotation(
        Placement(visible = true, transformation(origin = {41, 15}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
      DFIG_model.RV rv(ts_Wrm = 3) annotation(
        Placement(visible = true, transformation(origin = {23, 15}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Basic.Star star1 annotation(
        Placement(visible = true, transformation(origin = {-64, -38}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
      Modelica.Electrical.Analog.Basic.Ground ground annotation(
        Placement(visible = true, transformation(origin = {78, -50}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor reactivePowerSensor annotation(
        Placement(visible = true, transformation(origin = {49, 51}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor tacometro annotation(
        Placement(visible = true, transformation(origin = {14, 34}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Sources.CosineVoltage rede(V = fill(sqrt(2/3)*690, 3), f = fill(60, 3)) annotation(
        Placement(visible = true, transformation(origin = {-64, -24}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
      DFIG_model.MIT mit(D = 0) annotation(
        Placement(visible = true, transformation(origin = {74, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
        Placement(visible = true, transformation(origin = {-64, -50}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Basic.Star star annotation(
        Placement(visible = true, transformation(origin = {78, -38}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Sources.SignalCurrent FonteCorrente annotation(
        Placement(visible = true, transformation(origin = {78, -14}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
      AERO_model.TURBINA_model.CONTROL_tur CONTROL_tur annotation(
        Placement(visible = true, transformation(origin = {-12, 2}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = fill(10, 3)) annotation(
        Placement(visible = true, transformation(origin = {93, -14}, extent = {{-6, 5}, {6, -5}}, rotation = -90)));
      Modelica.Blocks.Sources.Ramp ramp(duration = 400, height = 20, offset = 4, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {-50, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(CONTROL_maq.iabcr, FonteCorrente.i) annotation(
        Line(points = {{42.9, -13.9}, {70.9, -13.9}}, color = {0, 0, 127}, thickness = 0.5));
      connect(Qref.y, rQs.Qsref) annotation(
        Line(points = {{53.6, 2}, {41, 2}, {41, 9.5}}, color = {0, 0, 127}));
      connect(reactivePowerSensor.plug_n, mit.plug_s) annotation(
        Line(points = {{53, 51}, {75, 51}, {75, 47}}, color = {0, 0, 255}));
      connect(CONTROL_tur.beta, turbina.beta) annotation(
        Line(points = {{-12, 10.8}, {-12, 27.8}}, color = {0, 0, 127}));
      connect(pll.VqPll, CONTROL_maq.Vqs_PLL) annotation(
        Line(points = {{6.6, -21.6}, {22.6, -21.6}}, color = {0, 0, 127}));
      connect(rQs.Qout, CONTROL_maq.Qs_esp) annotation(
        Line(points = {{35.5, 15}, {35.5, -3}}, color = {0, 0, 127}));
      connect(tacometro.w, CONTROL_maq.Wrm) annotation(
        Line(points = {{14, 29.6}, {14, -7.4}, {21, -7.4}}, color = {0, 0, 127}));
      connect(CONTROL_tur.Wrm_opt, rv.Wref) annotation(
        Line(points = {{-3, 2}, {25, 2}, {25, 9.5}, {22.8, 9.5}}, color = {0, 0, 127}));
      connect(reactivePowerSensor.plug_p, rede.plug_p) annotation(
        Line(points = {{45, 51}, {-64, 51}, {-64, -18}}, color = {0, 0, 255}));
      connect(pll.WePll, CONTROL_maq.We_PLL) annotation(
        Line(points = {{6.6, -18}, {21, -18}}, color = {0, 0, 127}));
      connect(reactivePowerSensor.reactivePower, rQs.Qs) annotation(
        Line(points = {{49, 46.6}, {49, 15}, {46.5, 15}}, color = {0, 0, 127}));
      connect(FonteCorrente.plug_p, resistor.plug_n) annotation(
        Line(points = {{78, -20}, {93, -20}}, color = {0, 0, 255}));
      connect(FonteCorrente.plug_n, mit.plug_r) annotation(
        Line(points = {{78, -8}, {78, 30}}, color = {0, 0, 255}));
      connect(ground1.p, star1.pin_n) annotation(
        Line(points = {{-64, -46}, {-64, -44}}, color = {0, 0, 255}));
      connect(rede.plug_n, star1.plug_p) annotation(
        Line(points = {{-64, -30}, {-64, -32}}, color = {0, 0, 255}));
      connect(rv.Te, CONTROL_maq.Te_esp) annotation(
        Line(points = {{28.5, 15}, {28.5, -3}}, color = {0, 0, 127}));
      connect(rede.plug_p, pll.vabc) annotation(
        Line(points = {{-64, -18}, {-6, -18}}, color = {0, 0, 255}));
      connect(tacometro.w, rv.W) annotation(
        Line(points = {{14, 29.6}, {14, 14.6}, {17.5, 14.6}}, color = {0, 0, 127}));
      connect(ground.p, star.pin_n) annotation(
        Line(points = {{78, -46}, {78, -44}}, color = {0, 0, 255}));
      connect(tacometro.flange, mit.eixo) annotation(
        Line(points = {{14, 38}, {63, 38}}));
      connect(FonteCorrente.plug_n, resistor.plug_p) annotation(
        Line(points = {{78, -8}, {93, -8}}, color = {0, 0, 255}));
      connect(pll.ThetaPll, CONTROL_maq.thetaE_PLL) annotation(
        Line(points = {{6.6, -14.4}, {22.6, -14.4}}, color = {0, 0, 127}));
      connect(star.plug_p, FonteCorrente.plug_p) annotation(
        Line(points = {{78, -32}, {78, -20}}, color = {0, 0, 255}));
      connect(ramp.y, turbina.Vw) annotation(
        Line(points = {{-39, 38}, {-21, 38}}, color = {0, 0, 127}));
      connect(ramp.y, CONTROL_tur.Vw) annotation(
        Line(points = {{-39, 38}, {-35, 38}, {-35, 2}, {-21, 2}}, color = {0, 0, 127}));
      connect(turbina.flange_Eixo, tacometro.flange) annotation(
        Line(points = {{-3, 38}, {14, 38}}));
    protected
      annotation(
        experiment(StartTime = 0, StopTime = 400, Tolerance = 1e-06, Interval = 0.0100025));
    end windturbineDFIG_simuR123;

    model windturbineDFIG_simuR4
      extends Modelica.Icons.Example;
      import pi = Modelica.Constants.pi;
      AERO_model.TURBINA_model.TURBINA turbina annotation(
        Placement(visible = true, transformation(origin = {-26, 38}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
      DFIG_model.CONTROL_maq CONTROL_maq annotation(
        Placement(visible = true, transformation(origin = {32, -14}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
      Modelica.Blocks.Sources.Step Qref(height = 0, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {58, 2}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
      AERO_model.DFIG_model.PLL pll annotation(
        Placement(visible = true, transformation(origin = {-22, -18}, extent = {{-6, 6}, {6, -6}}, rotation = 0)));
      DFIG_model.RQs rQs(ki_Q = 30) annotation(
        Placement(visible = true, transformation(origin = {41, 15}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
      DFIG_model.RV rv(ts_Wrm = 3) annotation(
        Placement(visible = true, transformation(origin = {23, 15}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Basic.Star star1 annotation(
        Placement(visible = true, transformation(origin = {-82, -38}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
      Modelica.Electrical.Analog.Basic.Ground ground annotation(
        Placement(visible = true, transformation(origin = {78, -50}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor reactivePowerSensor annotation(
        Placement(visible = true, transformation(origin = {49, 51}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor tacometro annotation(
        Placement(visible = true, transformation(origin = {12, 34}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Sources.CosineVoltage rede(V = fill(sqrt(2/3)*690, 3), f = fill(60, 3)) annotation(
        Placement(visible = true, transformation(origin = {-82, -24}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
      DFIG_model.MIT mit(D = 0) annotation(
        Placement(visible = true, transformation(origin = {74, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
        Placement(visible = true, transformation(origin = {-82, -50}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Basic.Star star annotation(
        Placement(visible = true, transformation(origin = {78, -38}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Sources.SignalCurrent FonteCorrente annotation(
        Placement(visible = true, transformation(origin = {78, -14}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
      AERO_model.TURBINA_model.CONTROL_tur CONTROL_tur annotation(
        Placement(visible = true, transformation(origin = {-26, 2}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = fill(10, 3)) annotation(
        Placement(visible = true, transformation(origin = {93, -14}, extent = {{-6, 5}, {6, -5}}, rotation = -90)));
      Modelica.Blocks.Sources.Ramp ramp(duration = 0, height = 5, offset = 12, startTime = 1) annotation(
        Placement(visible = true, transformation(origin = {-60, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sensors.PowerSensor powerSensor annotation(
        Placement(visible = true, transformation(origin = {-82, -6}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
    equation
      connect(CONTROL_maq.iabcr, FonteCorrente.i) annotation(
        Line(points = {{42.9, -13.9}, {70.9, -13.9}}, color = {0, 0, 127}, thickness = 0.5));
      connect(Qref.y, rQs.Qsref) annotation(
        Line(points = {{53.6, 2}, {41, 2}, {41, 9.5}}, color = {0, 0, 127}));
      connect(reactivePowerSensor.plug_n, mit.plug_s) annotation(
        Line(points = {{53, 51}, {75, 51}, {75, 47}}, color = {0, 0, 255}));
      connect(CONTROL_tur.beta, turbina.beta) annotation(
        Line(points = {{-26, 10.8}, {-26, 27.8}}, color = {0, 0, 127}));
      connect(pll.VqPll, CONTROL_maq.Vqs_PLL) annotation(
        Line(points = {{-15, -22}, {14.6, -22}, {14.6, -21.6}, {22.6, -21.6}}, color = {0, 0, 127}));
      connect(rQs.Qout, CONTROL_maq.Qs_esp) annotation(
        Line(points = {{35.5, 15}, {35.5, -3}}, color = {0, 0, 127}));
      connect(tacometro.w, CONTROL_maq.Wrm) annotation(
        Line(points = {{12, 30}, {12, -7.4}, {21, -7.4}}, color = {0, 0, 127}));
      connect(CONTROL_tur.Wrm_opt, rv.Wref) annotation(
        Line(points = {{-17, 2}, {24.8, 2}, {24.8, 9.5}, {22.8, 9.5}}, color = {0, 0, 127}));
      connect(pll.WePll, CONTROL_maq.We_PLL) annotation(
        Line(points = {{-15, -18}, {21, -18}}, color = {0, 0, 127}));
      connect(reactivePowerSensor.reactivePower, rQs.Qs) annotation(
        Line(points = {{49, 46.6}, {49, 15}, {46.5, 15}}, color = {0, 0, 127}));
      connect(FonteCorrente.plug_p, resistor.plug_n) annotation(
        Line(points = {{78, -20}, {93, -20}}, color = {0, 0, 255}));
      connect(FonteCorrente.plug_n, mit.plug_r) annotation(
        Line(points = {{78, -8}, {78, 30}}, color = {0, 0, 255}));
      connect(ground1.p, star1.pin_n) annotation(
        Line(points = {{-82, -46}, {-82, -44}}, color = {0, 0, 255}));
      connect(rede.plug_n, star1.plug_p) annotation(
        Line(points = {{-82, -30}, {-82, -32}}, color = {0, 0, 255}));
      connect(rv.Te, CONTROL_maq.Te_esp) annotation(
        Line(points = {{28.5, 15}, {28.5, -3}}, color = {0, 0, 127}));
      connect(rede.plug_p, pll.vabc) annotation(
        Line(points = {{-82, -18}, {-28, -18}}, color = {0, 0, 255}));
      connect(tacometro.w, rv.W) annotation(
        Line(points = {{12, 30}, {12, 14.6}, {17.5, 14.6}}, color = {0, 0, 127}));
      connect(ground.p, star.pin_n) annotation(
        Line(points = {{78, -46}, {78, -44}}, color = {0, 0, 255}));
      connect(tacometro.flange, mit.eixo) annotation(
        Line(points = {{12, 38}, {63, 38}}));
      connect(FonteCorrente.plug_n, resistor.plug_p) annotation(
        Line(points = {{78, -8}, {93, -8}}, color = {0, 0, 255}));
      connect(pll.ThetaPll, CONTROL_maq.thetaE_PLL) annotation(
        Line(points = {{-15, -14}, {14.6, -14}, {14.6, -14.4}, {22.6, -14.4}}, color = {0, 0, 127}));
      connect(star.plug_p, FonteCorrente.plug_p) annotation(
        Line(points = {{78, -32}, {78, -20}}, color = {0, 0, 255}));
      connect(turbina.flange_Eixo, mit.eixo) annotation(
        Line(points = {{-17, 38}, {64, 38}}));
      connect(ramp.y, turbina.Vw) annotation(
        Line(points = {{-49, 38}, {-34, 38}}, color = {0, 0, 127}));
      connect(ramp.y, CONTROL_tur.Vw) annotation(
        Line(points = {{-49, 38}, {-40, 38}, {-40, 2}, {-34, 2}}, color = {0, 0, 127}));
      connect(reactivePowerSensor.plug_p, powerSensor.pc) annotation(
        Line(points = {{46, 52}, {-82, 52}, {-82, 0}}, color = {0, 0, 255}));
      connect(powerSensor.nc, rede.plug_p) annotation(
        Line(points = {{-82, -12}, {-82, -18}}, color = {0, 0, 255}));
      connect(powerSensor.pv, rede.plug_p) annotation(
        Line(points = {{-76, -6}, {-70, -6}, {-70, -18}, {-82, -18}}, color = {0, 0, 255}));
      connect(powerSensor.nv, star1.plug_p) annotation(
        Line(points = {{-88, -6}, {-94, -6}, {-94, -32}, {-82, -32}}, color = {0, 0, 255}));
    protected
      annotation(
        experiment(StartTime = 0, StopTime = 25, Tolerance = 1e-06, Interval = 0.000208333));
    end windturbineDFIG_simuR4;

    model modelSimu
      extends Modelica.Icons.Example;
      import pi = Modelica.Constants.pi;
      Modelica.Electrical.Polyphase.Basic.Resistor R(R = fill(10, 3)) annotation(
        Placement(visible = true, transformation(origin = {87, -6}, extent = {{-6, 5}, {6, -5}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Basic.Star Y1 annotation(
        Placement(visible = true, transformation(origin = {-74, -30}, extent = {{6, -6}, {-6, 6}}, rotation = 90)));
      Modelica.Electrical.Polyphase.Sources.CosineVoltage Rede(V = fill(sqrt(2/3)*690, 3), f = fill(60, 3)) annotation(
        Placement(visible = true, transformation(origin = {-74, -16}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Basic.Star Y2 annotation(
        Placement(visible = true, transformation(origin = {72, -30}, extent = {{6, -6}, {-6, 6}}, rotation = 90)));
      AERO_model.DFIG_model.CONTROL_maq CONTROL_maq1 annotation(
        Placement(visible = true, transformation(origin = {14, -6}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Ground G1 annotation(
        Placement(visible = true, transformation(origin = {-74, -42}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      AERO_model.DFIG_model.MIT mit(D = 0) annotation(
        Placement(visible = true, transformation(origin = {68, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      AERO_model.DFIG_model.RQs rQs(ki_Q = 30) annotation(
        Placement(visible = true, transformation(origin = {23, 23}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
      AERO_model.DFIG_model.PLL pll1(zeta_PLL = 1) annotation(
        Placement(visible = true, transformation(origin = {-40, -10}, extent = {{-6, 6}, {6, -6}}, rotation = 0)));
      Modelica.Blocks.Sources.Step Qref(height = 0, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {40, 14}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sources.SignalCurrent FC annotation(
        Placement(visible = true, transformation(origin = {72, -6}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
      Modelica.Electrical.Analog.Basic.Ground G2 annotation(
        Placement(visible = true, transformation(origin = {72, -42}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor Wmed annotation(
        Placement(visible = true, transformation(origin = {-20, 30}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
      AERO_model.DFIG_model.RV rv(ts_Wrm = 1, zeta_Wrm = .5) annotation(
        Placement(visible = true, transformation(origin = {5, 23}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor Qmed annotation(
        Placement(visible = true, transformation(origin = {47, 47}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sensors.PowerSensor Pmed annotation(
        Placement(visible = true, transformation(origin = {-74, -2}, extent = {{4, -4}, {-4, 4}}, rotation = 90)));
      Modelica.Blocks.Sources.Step Vw(height = 0, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {-64, 34}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      AERO_model.TURBINA_model.CONTROL_tur CONTROL_tur annotation(
        Placement(visible = true, transformation(origin = {-40, 14}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      AERO_model.TURBINA_model.TURBINA turbina(ts = 1) annotation(
        Placement(visible = true, transformation(origin = {-40, 34}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
    equation
      connect(Y2.plug_p, FC.plug_p) annotation(
        Line(points = {{72, -24}, {72, -12}}, color = {0, 0, 255}));
      connect(pll1.WePll, CONTROL_maq1.We_PLL) annotation(
        Line(points = {{-33.4, -10}, {2.6, -10}}, color = {0, 0, 255}));
      connect(G1.p, Y1.pin_n) annotation(
        Line(points = {{-74, -38}, {-74, -36}}, color = {0, 0, 255}));
      connect(Rede.plug_p, pll1.vabc) annotation(
        Line(points = {{-74, -10}, {-47, -10}}, color = {0, 0, 255}));
      connect(Rede.plug_n, Y1.plug_p) annotation(
        Line(points = {{-74, -22}, {-74, -24}}, color = {0, 0, 255}));
      connect(Wmed.w, CONTROL_maq1.Wrm) annotation(
        Line(points = {{-20, 26}, {-20, 1}, {3, 1}}, color = {0, 0, 255}));
      connect(FC.plug_n, R.plug_p) annotation(
        Line(points = {{72, 0}, {87, 0}}, color = {0, 0, 255}));
      connect(FC.plug_n, mit.plug_r) annotation(
        Line(points = {{72, 0}, {72, 30}}, color = {0, 0, 255}));
      connect(FC.plug_p, R.plug_n) annotation(
        Line(points = {{72, -12}, {87, -12}}, color = {0, 0, 255}));
      connect(G2.p, Y2.pin_n) annotation(
        Line(points = {{72, -38}, {72, -36}}, color = {0, 0, 255}));
      connect(Wmed.w, rv.W) annotation(
        Line(points = {{-20, 26}, {-20, 23}, {-0.5, 23}}, color = {0, 0, 255}));
      connect(rQs.Qout, CONTROL_maq1.Qs_esp) annotation(
        Line(points = {{17.5, 23}, {17.5, 5}, {18, 5}}, color = {0, 0, 255}));
      connect(rv.Te, CONTROL_maq1.Te_esp) annotation(
        Line(points = {{10.5, 23}, {10.5, 5}, {10, 5}}, color = {0, 0, 255}));
      connect(Qmed.reactivePower, rQs.Qs) annotation(
        Line(points = {{47, 43}, {47, 23}, {28.5, 23}}, color = {0, 0, 255}));
      connect(Qmed.plug_n, mit.plug_s) annotation(
        Line(points = {{51, 47}, {69, 47}}, color = {0, 0, 255}));
      connect(Pmed.nv, Rede.plug_n) annotation(
        Line(points = {{-70, -2}, {-84, -2}, {-84, -22}, {-74, -22}}, color = {0, 0, 255}));
      connect(Pmed.pv, Rede.plug_p) annotation(
        Line(points = {{-78, -2}, {-66, -2}, {-66, -10}, {-74, -10}}, color = {0, 0, 255}));
      connect(Pmed.pc, Qmed.plug_p) annotation(
        Line(points = {{-74, 2}, {-74, 47}, {43, 47}}, color = {0, 0, 255}));
      connect(Rede.plug_p, Pmed.nc) annotation(
        Line(points = {{-74, -10}, {-74, -6}}, color = {0, 0, 255}));
      connect(Qref.y, rQs.Qsref) annotation(
        Line(points = {{35.6, 14}, {22.6, 14}, {22.6, 17.5}}, color = {0, 0, 255}));
      connect(pll1.VqPll, CONTROL_maq1.Vqs_PLL) annotation(
        Line(points = {{-33.4, -13.6}, {2.6, -13.6}}, color = {0, 0, 255}));
      connect(pll1.ThetaPll, CONTROL_maq1.thetaE_PLL) annotation(
        Line(points = {{-33.4, -6.4}, {2.6, -6.4}}, color = {0, 0, 255}));
      connect(CONTROL_tur.beta, turbina.beta) annotation(
        Line(points = {{-40, 21}, {-40, 27}}, color = {0, 0, 127}));
      connect(CONTROL_tur.Wrm_opt, rv.Wref) annotation(
        Line(points = {{-33, 14}, {5, 14}, {5, 17.5}}, color = {0, 0, 127}));
      connect(Vw.y, turbina.Vw) annotation(
        Line(points = {{-59.6, 34}, {-46.6, 34}}, color = {0, 0, 127}));
      connect(Vw.y, CONTROL_tur.Vw) annotation(
        Line(points = {{-60, 34}, {-54, 34}, {-54, 14}, {-47, 14}}, color = {0, 0, 127}));
      connect(turbina.flange_Eixo, Wmed.flange) annotation(
        Line(points = {{-33, 34}, {-20, 34}}));
  connect(CONTROL_maq1.iabcr, FC.i) annotation(
        Line(points = {{24, -6}, {64, -6}}, color = {0, 0, 127}, thickness = 0.5));
  connect(mit.eixo, Wmed.flange) annotation(
        Line(points = {{58, 38}, {-20, 38}, {-20, 34}}));
    protected
      annotation(
        experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.000833333),
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(origin = {4, 23}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-8, 7}, {8, -7}}), Rectangle(origin = {24, 23}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-8, 7}, {8, -7}}), Rectangle(origin = {68, 38}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-12, 10}, {12, -10}}), Rectangle(origin = {-40, -10}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-10, 8}, {10, -8}}), Rectangle(origin = {14, -5}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-18, 13}, {18, -13}}), Rectangle(origin = {14, -20}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-18, 2}, {18, -2}}), Rectangle(origin = {-40, -20}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-10, 2}, {10, -2}}), Rectangle(origin = {68, 50}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-12, 2}, {12, -2}}), Rectangle(origin = {4, 32}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-8, 2}, {8, -2}}), Rectangle(origin = {24, 32}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-8, 2}, {8, -2}}), Text(origin = {68, 50}, textColor = {0, 0, 255}, extent = {{-10, 2}, {10, -2}}, textString = "MODELO MIT", textStyle = {TextStyle.Italic}), Text(origin = {14, -20}, textColor = {0, 0, 255}, extent = {{-16, 2}, {16, -2}}, textString = "MODELO EQ CORRENTE", textStyle = {TextStyle.Italic, TextStyle.Italic}), Text(origin = {-40, -20}, textColor = {0, 0, 255}, extent = {{-8, 2}, {8, -2}}, textString = "MODELO PLL", textStyle = {TextStyle.Italic}), Text(origin = {4, 32}, textColor = {0, 0, 255}, extent = {{-6, 2}, {6, -2}}, textString = "MODELO RV", textStyle = {TextStyle.Italic}), Text(origin = {24, 32}, textColor = {0, 0, 255}, extent = {{-6, 2}, {6, -2}}, textString = "MODELO RQ", textStyle = {TextStyle.Italic, TextStyle.Italic}), Text(extent = {{-22, 50}, {-22, 50}}, textString = "text"), Text(origin = {13, 36}, textColor = {85, 87, 83}, extent = {{-5, 2}, {5, -2}}, textString = "Tm, Wrm"), Text(origin = {-76, 22}, rotation = -90, textColor = {85, 87, 83}, extent = {{-8, 2}, {8, -2}}, textString = "vabcs, iabcs"), Text(origin = {74, 14}, rotation = -90, textColor = {85, 87, 83}, extent = {{-8, 2}, {8, -2}}, textString = "vabcr, iabcr"), Text(origin = {-23, 24}, rotation = -90, textColor = {85, 87, 83}, extent = {{-5, 2}, {5, -2}}, textString = "Wrm,med"), Text(origin = {6, 12}, rotation = 180, textColor = {85, 87, 83}, extent = {{-4, 2}, {4, -2}}, textString = "Te,reg"), Text(origin = {22, 12}, rotation = 180, textColor = {85, 87, 83}, extent = {{-4, 2}, {4, -2}}, textString = "Qs,reg"), Text(origin = {49, 27}, rotation = -90, textColor = {85, 87, 83}, extent = {{-4, 2}, {4, -2}}, textString = "Qs,med"), Text(origin = {-16, -4}, rotation = 180, textColor = {85, 87, 83}, extent = {{-4, 2}, {4, -2}}, textString = "Vq,PLL"), Text(origin = {-16, -8}, rotation = 180, textColor = {85, 87, 83}, extent = {{-4, 2}, {4, -2}}, textString = "We,PLL"), Text(origin = {-16, -12}, rotation = 180, textColor = {85, 87, 83}, extent = {{-4, 2}, {4, -2}}, textString = "Th,PLL"), Text(origin = {48, -4}, rotation = 180, textColor = {85, 87, 83}, extent = {{-4, 2}, {4, -2}}, textString = "iabcr"), Text(origin = {-11, 16}, rotation = 180, textColor = {85, 87, 83}, extent = {{-5, 2}, {5, -2}}, textString = "Wrm,ref"), Rectangle(origin = {-40, 4}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-10, 2}, {10, -2}}), Rectangle(origin = {-40, 14}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-10, 8}, {10, -8}}), Text(origin = {-40, 4}, textColor = {0, 0, 255}, extent = {{-8, 2}, {8, -2}}, textString = "MODELO REG", textStyle = {TextStyle.Italic}), Rectangle(origin = {-40, 44}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-10, 2}, {10, -2}}), Rectangle(origin = {-40, 34}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-10, 8}, {10, -8}}), Text(origin = {-40, 44}, textColor = {0, 0, 255}, extent = {{-8, 2}, {8, -2}}, textString = "MODELO TUR", textStyle = {TextStyle.Italic}), Text(origin = {-46, 24}, rotation = 180, textColor = {85, 87, 83}, extent = {{-6, 2}, {6, -2}}, textString = "Beta,ref")}));
    end modelSimu;
    
    model modelSimu2
      extends Modelica.Icons.Example;
      import pi = Modelica.Constants.pi;
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor Wmed annotation(
        Placement(visible = true, transformation(origin = {-28, 32}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
      AERO_model.DFIG_model.RV rv(ts_Wrm = 1, zeta_Wrm = .5) annotation(
        Placement(visible = true, transformation(origin = {-21, 17}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
      Modelica.Blocks.Sources.Step Vw(height = 0, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {-64, 36}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      AERO_model.TURBINA_model.CONTROL_tur CONTROL_tur annotation(
        Placement(visible = true, transformation(origin = {-40, 8}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      AERO_model.TURBINA_model.TURBINA turbina(ts = 1) annotation(
        Placement(visible = true, transformation(origin = {-40, 36}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  AERO_model.DFIG_model.MIT mit(D = 0) annotation(
        Placement(visible = true, transformation(origin = {-10, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(Wmed.w, rv.W) annotation(
        Line(points = {{-28, 28}, {-28, 17}, {-26.5, 17}}, color = {0, 0, 255}));
      connect(CONTROL_tur.beta, turbina.beta) annotation(
        Line(points = {{-40, 15}, {-40, 29}}, color = {0, 0, 127}));
      connect(Vw.y, turbina.Vw) annotation(
        Line(points = {{-60, 36}, {-47, 36}}, color = {0, 0, 127}));
      connect(Vw.y, CONTROL_tur.Vw) annotation(
        Line(points = {{-60, 36}, {-54, 36}, {-54, 8}, {-47, 8}}, color = {0, 0, 127}));
      connect(turbina.flange_Eixo, Wmed.flange) annotation(
        Line(points = {{-33, 36}, {-28, 36}}));
  connect(Wmed.flange, mit.eixo) annotation(
        Line(points = {{-28, 36}, {-21, 36}}));
  connect(CONTROL_tur.Wrm_opt, rv.Wref) annotation(
        Line(points = {{-33.4, 8}, {-21, 8}, {-21, 11.5}}, color = {0, 0, 127}));
    protected
      annotation(
        experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.000833333),
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Text(extent = {{-22, 50}, {-22, 50}}, textString = "text"), Rectangle(origin = {-40, 8}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-10, 8}, {10, -8}})}));
    end modelSimu2;
  end Examples;
  annotation(
    uses(Modelica(version = "4.0.0")));
end AERO_model;