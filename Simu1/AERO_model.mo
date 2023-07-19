package AERO_model
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

    model CONTROL_maqeDFIG
      extends Modelica.Icons.Example;
      import pi = Modelica.Constants.pi;
      Modelica.Blocks.Math.Add add annotation(
        Placement(visible = true, transformation(origin = {-46, 40}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Basic.Resistor resistor1(R = fill(10, 3)) annotation(
        Placement(visible = true, transformation(origin = {67, 2}, extent = {{-6, 5}, {6, -5}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor reactivePowerSensor1 annotation(
        Placement(visible = true, transformation(origin = {31, 53}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Basic.Star star2 annotation(
        Placement(visible = true, transformation(origin = {-36, -22}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Sources.CosineVoltage cosineVoltage(V = fill(sqrt(2/3)*690, 3), f = fill(60, 3)) annotation(
        Placement(visible = true, transformation(origin = {-36, -8}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Basic.Star star3 annotation(
        Placement(visible = true, transformation(origin = {52, -22}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
      Modelica.Blocks.Sources.Step step(height = 10.16e3, offset = 0, startTime = 1) annotation(
        Placement(visible = true, transformation(origin = {-64, 52}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Blocks.Sources.Step step3(height = 0, offset = 0) annotation(
        Placement(visible = true, transformation(origin = {100, 32}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
      AERO_model.DFIG_model.CONTROL_maq CONTROL_maq1 annotation(
        Placement(visible = true, transformation(origin = {14, 2}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Ground ground2 annotation(
        Placement(visible = true, transformation(origin = {-36, -34}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Blocks.Sources.Step step1(height = -10.16e3, offset = 0, startTime = 6) annotation(
        Placement(visible = true, transformation(origin = {-64, 38}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      AERO_model.DFIG_model.MIT mit(D = 0) annotation(
        Placement(visible = true, transformation(origin = {48, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Add add2 annotation(
        Placement(visible = true, transformation(origin = {-64, 20}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      AERO_model.DFIG_model.RQs rQs(ki_Q = 30) annotation(
        Placement(visible = true, transformation(origin = {23, 25}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
      Modelica.Blocks.Math.Add add1 annotation(
        Placement(visible = true, transformation(origin = {80, 20}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
      AERO_model.DFIG_model.PLL pll1(zeta_PLL = 1) annotation(
        Placement(visible = true, transformation(origin = {-18, -2}, extent = {{-6, 6}, {6, -6}}, rotation = 0)));
      Modelica.Blocks.Sources.Step step2(height = 0, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {100, 18}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
      Modelica.Blocks.Sources.Step step4(height = 0, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {-82, 18}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.Torque torque annotation(
        Placement(visible = true, transformation(origin = {-28, 40}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sources.SignalCurrent signalCurrent annotation(
        Placement(visible = true, transformation(origin = {52, 2}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
      Modelica.Blocks.Sources.Step step5(height = 0, offset = 185) annotation(
        Placement(visible = true, transformation(origin = {-82, 32}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Ground ground3 annotation(
        Placement(visible = true, transformation(origin = {52, -34}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation(
        Placement(visible = true, transformation(origin = {-10, 36}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
      AERO_model.DFIG_model.RV rv(ts_Wrm = 1, zeta_Wrm = .5) annotation(
        Placement(visible = true, transformation(origin = {5, 25}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sensors.PowerSensor powerSensor annotation(
        Placement(visible = true, transformation(origin = {-36, 10}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
    equation
      connect(step3.y, add1.u1) annotation(
        Line(points = {{95.6, 32}, {91.6, 32}, {91.6, 22}, {83.6, 22}}, color = {0, 0, 127}));
      connect(pll1.VqPll, CONTROL_maq1.Vqs_PLL) annotation(
        Line(points = {{-11.4, -5.6}, {3.6, -5.6}}, color = {0, 0, 255}));
      connect(step5.y, add2.u1) annotation(
        Line(points = {{-77.6, 32}, {-73.6, 32}, {-73.6, 22}, {-67.6, 22}}, color = {0, 0, 127}));
      connect(torque.flange, mit.eixo) annotation(
        Line(points = {{-24, 40}, {38, 40}}, color = {0, 0, 255}));
      connect(star3.plug_p, signalCurrent.plug_p) annotation(
        Line(points = {{52, -16}, {52, -4}}, color = {0, 0, 255}));
      connect(pll1.WePll, CONTROL_maq1.We_PLL) annotation(
        Line(points = {{-11.4, -2}, {3.6, -2}}, color = {0, 0, 255}));
      connect(ground2.p, star2.pin_n) annotation(
        Line(points = {{-36, -30}, {-36, -28}}, color = {0, 0, 255}));
      connect(add.y, torque.tau) annotation(
        Line(points = {{-41.6, 40}, {-32.6, 40}}, color = {0, 0, 127}));
      connect(step.y, add.u1) annotation(
        Line(points = {{-59.6, 52}, {-55.6, 52}, {-55.6, 42}, {-49.6, 42}}, color = {0, 0, 127}));
      connect(reactivePowerSensor1.plug_n, mit.plug_s) annotation(
        Line(points = {{35, 53}, {49, 53}, {49, 49}}, color = {0, 0, 255}));
      connect(cosineVoltage.plug_p, pll1.vabc) annotation(
        Line(points = {{-36, -2}, {-24, -2}}, color = {0, 0, 255}));
      connect(pll1.ThetaPll, CONTROL_maq1.thetaE_PLL) annotation(
        Line(points = {{-11.4, 1.6}, {3.6, 1.6}}, color = {0, 0, 255}));
      connect(CONTROL_maq1.iabcr, signalCurrent.i) annotation(
        Line(points = {{24.9, 2.1}, {44.9, 2.1}}, color = {0, 0, 255}, thickness = 0.5));
      connect(step4.y, add2.u2) annotation(
        Line(points = {{-77.6, 18}, {-68.6, 18}}, color = {0, 0, 127}));
      connect(cosineVoltage.plug_n, star2.plug_p) annotation(
        Line(points = {{-36, -14}, {-36, -16}}, color = {0, 0, 255}));
      connect(speedSensor.w, CONTROL_maq1.Wrm) annotation(
        Line(points = {{-10, 31.6}, {-10, 8.6}, {3, 8.6}}, color = {0, 0, 255}));
      connect(step1.y, add.u2) annotation(
        Line(points = {{-59.6, 38}, {-50.6, 38}}, color = {0, 0, 127}));
      connect(reactivePowerSensor1.reactivePower, rQs.Qs) annotation(
        Line(points = {{31, 48.6}, {31, 25}, {28.5, 25}}, color = {0, 0, 255}));
      connect(signalCurrent.plug_n, resistor1.plug_p) annotation(
        Line(points = {{52, 8}, {67, 8}}, color = {0, 0, 255}));
      connect(signalCurrent.plug_n, mit.plug_r) annotation(
        Line(points = {{52, 8}, {52, 32}}, color = {0, 0, 255}));
      connect(signalCurrent.plug_p, resistor1.plug_n) annotation(
        Line(points = {{52, -4}, {67, -4}}, color = {0, 0, 255}));
      connect(speedSensor.flange, mit.eixo) annotation(
        Line(points = {{-10, 40}, {37, 40}}));
      connect(ground3.p, star3.pin_n) annotation(
        Line(points = {{52, -30}, {52, -28}}, color = {0, 0, 255}));
      connect(step2.y, add1.u2) annotation(
        Line(points = {{95.6, 18}, {84.6, 18}}, color = {0, 0, 127}));
      connect(speedSensor.w, rv.W) annotation(
        Line(points = {{-10, 32}, {-10, 25}, {-0.5, 25}}, color = {0, 0, 127}));
      connect(add2.y, rv.Wref) annotation(
        Line(points = {{-60, 20}, {-29, 20}, {-29, 19.5}, {5, 19.5}}, color = {0, 0, 127}));
      connect(rQs.Qsref, add1.y) annotation(
        Line(points = {{23, 19.5}, {43, 19.5}, {43, 20}, {76, 20}}, color = {0, 0, 127}));
      connect(rQs.Qout, CONTROL_maq1.Qs_esp) annotation(
        Line(points = {{17.5, 25}, {17.5, 14}, {18, 14}}, color = {0, 0, 127}));
      connect(rv.Te, CONTROL_maq1.Te_esp) annotation(
        Line(points = {{10.5, 25}, {10.5, 14}, {10, 14}}, color = {0, 0, 127}));
      connect(cosineVoltage.plug_p, powerSensor.nc) annotation(
        Line(points = {{-36, -2}, {-36, 6}}, color = {0, 0, 255}));
      connect(powerSensor.pc, reactivePowerSensor1.plug_p) annotation(
        Line(points = {{-36, 14}, {-36, 54}, {28, 54}}, color = {0, 0, 255}));
      connect(powerSensor.pv, cosineVoltage.plug_p) annotation(
        Line(points = {{-32, 10}, {-28, 10}, {-28, -2}, {-36, -2}}, color = {0, 0, 255}));
      connect(powerSensor.nv, cosineVoltage.plug_n) annotation(
        Line(points = {{-40, 10}, {-46, 10}, {-46, -14}, {-36, -14}}, color = {0, 0, 255}));
    protected
      annotation(
        experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.000833333),
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})));
    end CONTROL_maqeDFIG;

    model CONTROL_maqeDFIG_comzeros
      extends Modelica.Icons.Example;
      import pi = Modelica.Constants.pi;
      Modelica.Blocks.Math.Add add annotation(
        Placement(visible = true, transformation(origin = {-46, 40}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Basic.Resistor resistor1(R = fill(10, 3)) annotation(
        Placement(visible = true, transformation(origin = {67, 2}, extent = {{-6, 5}, {6, -5}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor reactivePowerSensor1 annotation(
        Placement(visible = true, transformation(origin = {31, 53}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Basic.Star star2 annotation(
        Placement(visible = true, transformation(origin = {-36, -22}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Sources.CosineVoltage cosineVoltage(V = fill(sqrt(2/3)*690, 3), f = fill(60, 3)) annotation(
        Placement(visible = true, transformation(origin = {-36, -8}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Basic.Star star3 annotation(
        Placement(visible = true, transformation(origin = {52, -22}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
      Modelica.Blocks.Sources.Step step(height = 10.5e3, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {-64, 52}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Blocks.Sources.Step step3(height = 0, offset = 0) annotation(
        Placement(visible = true, transformation(origin = {100, 32}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
      AERO_model.DFIG_model.CONTROL_maq CONTROL_maq1 annotation(
        Placement(visible = true, transformation(origin = {14, 2}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Ground ground2 annotation(
        Placement(visible = true, transformation(origin = {-36, -34}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Blocks.Sources.Step step1(height = 0, offset = 0, startTime = 6) annotation(
        Placement(visible = true, transformation(origin = {-64, 38}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      AERO_model.DFIG_model.MIT mit(D = 0) annotation(
        Placement(visible = true, transformation(origin = {48, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Add add2 annotation(
        Placement(visible = true, transformation(origin = {-64, 20}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      AERO_model.DFIG_model.RQs rQs(ki_Q = 10) annotation(
        Placement(visible = true, transformation(origin = {23, 25}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
      Modelica.Blocks.Math.Add add1 annotation(
        Placement(visible = true, transformation(origin = {80, 20}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
      AERO_model.DFIG_model.PLL pll1(zeta_PLL = 1) annotation(
        Placement(visible = true, transformation(origin = {-18, -2}, extent = {{-6, 6}, {6, -6}}, rotation = 0)));
      Modelica.Blocks.Sources.Step step2(height = 0, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {100, 18}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
      Modelica.Blocks.Sources.Step step4(height = 10, offset = 0, startTime = 1) annotation(
        Placement(visible = true, transformation(origin = {-82, 18}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.Torque torque annotation(
        Placement(visible = true, transformation(origin = {-28, 40}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sources.SignalCurrent signalCurrent annotation(
        Placement(visible = true, transformation(origin = {52, 2}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
      Modelica.Blocks.Sources.Step step5(height = 0, offset = 180) annotation(
        Placement(visible = true, transformation(origin = {-82, 32}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Ground ground3 annotation(
        Placement(visible = true, transformation(origin = {52, -34}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation(
        Placement(visible = true, transformation(origin = {-10, 36}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
      AERO_model.DFIG_model.RV_comZeros rV_comZeros(ts_Wrm = 1) annotation(
        Placement(visible = true, transformation(origin = {5, 25}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    equation
      connect(step3.y, add1.u1) annotation(
        Line(points = {{95.6, 32}, {91.6, 32}, {91.6, 22}, {83.6, 22}}, color = {0, 0, 127}));
      connect(pll1.VqPll, CONTROL_maq1.Vqs_PLL) annotation(
        Line(points = {{-11.4, -5.6}, {3.6, -5.6}}, color = {0, 0, 255}));
      connect(step5.y, add2.u1) annotation(
        Line(points = {{-77.6, 32}, {-73.6, 32}, {-73.6, 22}, {-67.6, 22}}, color = {0, 0, 127}));
      connect(torque.flange, mit.eixo) annotation(
        Line(points = {{-24, 40}, {38, 40}}, color = {0, 0, 255}));
      connect(star3.plug_p, signalCurrent.plug_p) annotation(
        Line(points = {{52, -16}, {52, -4}}, color = {0, 0, 255}));
      connect(pll1.WePll, CONTROL_maq1.We_PLL) annotation(
        Line(points = {{-11.4, -2}, {3.6, -2}}, color = {0, 0, 255}));
      connect(ground2.p, star2.pin_n) annotation(
        Line(points = {{-36, -30}, {-36, -28}}, color = {0, 0, 255}));
      connect(add.y, torque.tau) annotation(
        Line(points = {{-41.6, 40}, {-32.6, 40}}, color = {0, 0, 127}));
      connect(step.y, add.u1) annotation(
        Line(points = {{-59.6, 52}, {-55.6, 52}, {-55.6, 42}, {-49.6, 42}}, color = {0, 0, 127}));
      connect(reactivePowerSensor1.plug_n, mit.plug_s) annotation(
        Line(points = {{35, 53}, {49, 53}, {49, 49}}, color = {0, 0, 255}));
      connect(reactivePowerSensor1.plug_p, cosineVoltage.plug_p) annotation(
        Line(points = {{27, 53}, {-36, 53}, {-36, -2}}, color = {0, 0, 255}));
      connect(cosineVoltage.plug_p, pll1.vabc) annotation(
        Line(points = {{-36, -2}, {-24, -2}}, color = {0, 0, 255}));
      connect(pll1.ThetaPll, CONTROL_maq1.thetaE_PLL) annotation(
        Line(points = {{-11.4, 1.6}, {3.6, 1.6}}, color = {0, 0, 255}));
      connect(CONTROL_maq1.iabcr, signalCurrent.i) annotation(
        Line(points = {{24.9, 2.1}, {44.9, 2.1}}, color = {0, 0, 255}, thickness = 0.5));
      connect(step4.y, add2.u2) annotation(
        Line(points = {{-77.6, 18}, {-68.6, 18}}, color = {0, 0, 127}));
      connect(cosineVoltage.plug_n, star2.plug_p) annotation(
        Line(points = {{-36, -14}, {-36, -16}}, color = {0, 0, 255}));
      connect(speedSensor.w, CONTROL_maq1.Wrm) annotation(
        Line(points = {{-10, 31.6}, {-10, 8.6}, {3, 8.6}}, color = {0, 0, 255}));
      connect(step1.y, add.u2) annotation(
        Line(points = {{-59.6, 38}, {-50.6, 38}}, color = {0, 0, 127}));
      connect(reactivePowerSensor1.reactivePower, rQs.Qs) annotation(
        Line(points = {{31, 48.6}, {31, 25}, {28.5, 25}}, color = {0, 0, 255}));
      connect(signalCurrent.plug_n, resistor1.plug_p) annotation(
        Line(points = {{52, 8}, {67, 8}}, color = {0, 0, 255}));
      connect(signalCurrent.plug_n, mit.plug_r) annotation(
        Line(points = {{52, 8}, {52, 32}}, color = {0, 0, 255}));
      connect(signalCurrent.plug_p, resistor1.plug_n) annotation(
        Line(points = {{52, -4}, {67, -4}}, color = {0, 0, 255}));
      connect(speedSensor.flange, mit.eixo) annotation(
        Line(points = {{-10, 40}, {37, 40}}));
      connect(ground3.p, star3.pin_n) annotation(
        Line(points = {{52, -30}, {52, -28}}, color = {0, 0, 255}));
      connect(step2.y, add1.u2) annotation(
        Line(points = {{95.6, 18}, {84.6, 18}}, color = {0, 0, 127}));
      connect(rQs.Qsref, add1.y) annotation(
        Line(points = {{23, 19.5}, {43, 19.5}, {43, 20}, {76, 20}}, color = {0, 0, 127}));
      connect(rQs.Qout, CONTROL_maq1.Qs_esp) annotation(
        Line(points = {{17.5, 25}, {17.5, 14}, {18, 14}}, color = {0, 0, 127}));
      connect(speedSensor.w, rV_comZeros.W) annotation(
        Line(points = {{-10, 32}, {-10, 26}, {0, 26}}, color = {0, 0, 127}));
      connect(add2.y, rV_comZeros.Wref) annotation(
        Line(points = {{-60, 20}, {6, 20}}, color = {0, 0, 127}));
      connect(rV_comZeros.Te, CONTROL_maq1.Te_esp) annotation(
        Line(points = {{10, 26}, {10, 14}}, color = {0, 0, 127}));
    protected
      annotation(
        experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.000833333),
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})));
    end CONTROL_maqeDFIG_comzeros;

    model CONTROL_maqeDFIG_semzeros
      extends Modelica.Icons.Example;
      import pi = Modelica.Constants.pi;
      Modelica.Blocks.Math.Add add annotation(
        Placement(visible = true, transformation(origin = {-46, 40}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Basic.Resistor resistor1(R = fill(10, 3)) annotation(
        Placement(visible = true, transformation(origin = {67, 2}, extent = {{-6, 5}, {6, -5}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor reactivePowerSensor1 annotation(
        Placement(visible = true, transformation(origin = {31, 53}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Basic.Star star2 annotation(
        Placement(visible = true, transformation(origin = {-36, -22}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Sources.CosineVoltage cosineVoltage(V = fill(sqrt(2/3)*690, 3), f = fill(60, 3)) annotation(
        Placement(visible = true, transformation(origin = {-36, -8}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Basic.Star star3 annotation(
        Placement(visible = true, transformation(origin = {52, -22}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
      Modelica.Blocks.Sources.Step step(height = 10.5e3, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {-64, 52}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Blocks.Sources.Step step3(height = 0, offset = 0) annotation(
        Placement(visible = true, transformation(origin = {100, 32}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
      AERO_model.DFIG_model.CONTROL_maq CONTROL_maq1 annotation(
        Placement(visible = true, transformation(origin = {14, 2}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Ground ground2 annotation(
        Placement(visible = true, transformation(origin = {-36, -34}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Blocks.Sources.Step step1(height = -10.16e3, offset = 0, startTime = 6) annotation(
        Placement(visible = true, transformation(origin = {-64, 38}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      AERO_model.DFIG_model.MIT mit(D = 0) annotation(
        Placement(visible = true, transformation(origin = {48, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Math.Add add2 annotation(
        Placement(visible = true, transformation(origin = {-64, 20}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      AERO_model.DFIG_model.RQs rQs(ki_Q = 10) annotation(
        Placement(visible = true, transformation(origin = {23, 25}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
      Modelica.Blocks.Math.Add add1 annotation(
        Placement(visible = true, transformation(origin = {80, 20}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
      AERO_model.DFIG_model.PLL pll1(zeta_PLL = 1) annotation(
        Placement(visible = true, transformation(origin = {-18, -2}, extent = {{-6, 6}, {6, -6}}, rotation = 0)));
      Modelica.Blocks.Sources.Step step2(height = 0, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {100, 18}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
      Modelica.Blocks.Sources.Step step4(height = 10, offset = 0, startTime = 1) annotation(
        Placement(visible = true, transformation(origin = {-82, 18}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.Torque torque annotation(
        Placement(visible = true, transformation(origin = {-28, 40}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sources.SignalCurrent signalCurrent annotation(
        Placement(visible = true, transformation(origin = {52, 2}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
      Modelica.Blocks.Sources.Step step5(height = 0, offset = 180) annotation(
        Placement(visible = true, transformation(origin = {-82, 32}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Ground ground3 annotation(
        Placement(visible = true, transformation(origin = {52, -34}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation(
        Placement(visible = true, transformation(origin = {-10, 36}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
      AERO_model.DFIG_model.RV rv(ts_Wrm = 1) annotation(
        Placement(visible = true, transformation(origin = {5, 25}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
    equation
      connect(step3.y, add1.u1) annotation(
        Line(points = {{95.6, 32}, {91.6, 32}, {91.6, 22}, {83.6, 22}}, color = {0, 0, 127}));
      connect(pll1.VqPll, CONTROL_maq1.Vqs_PLL) annotation(
        Line(points = {{-11.4, -5.6}, {3.6, -5.6}}, color = {0, 0, 255}));
      connect(step5.y, add2.u1) annotation(
        Line(points = {{-77.6, 32}, {-73.6, 32}, {-73.6, 22}, {-67.6, 22}}, color = {0, 0, 127}));
      connect(torque.flange, mit.eixo) annotation(
        Line(points = {{-24, 40}, {38, 40}}, color = {0, 0, 255}));
      connect(star3.plug_p, signalCurrent.plug_p) annotation(
        Line(points = {{52, -16}, {52, -4}}, color = {0, 0, 255}));
      connect(pll1.WePll, CONTROL_maq1.We_PLL) annotation(
        Line(points = {{-11.4, -2}, {3.6, -2}}, color = {0, 0, 255}));
      connect(ground2.p, star2.pin_n) annotation(
        Line(points = {{-36, -30}, {-36, -28}}, color = {0, 0, 255}));
      connect(add.y, torque.tau) annotation(
        Line(points = {{-41.6, 40}, {-32.6, 40}}, color = {0, 0, 127}));
      connect(step.y, add.u1) annotation(
        Line(points = {{-59.6, 52}, {-55.6, 52}, {-55.6, 42}, {-49.6, 42}}, color = {0, 0, 127}));
      connect(reactivePowerSensor1.plug_n, mit.plug_s) annotation(
        Line(points = {{35, 53}, {49, 53}, {49, 49}}, color = {0, 0, 255}));
      connect(reactivePowerSensor1.plug_p, cosineVoltage.plug_p) annotation(
        Line(points = {{27, 53}, {-36, 53}, {-36, -2}}, color = {0, 0, 255}));
      connect(cosineVoltage.plug_p, pll1.vabc) annotation(
        Line(points = {{-36, -2}, {-24, -2}}, color = {0, 0, 255}));
      connect(pll1.ThetaPll, CONTROL_maq1.thetaE_PLL) annotation(
        Line(points = {{-11.4, 1.6}, {3.6, 1.6}}, color = {0, 0, 255}));
      connect(CONTROL_maq1.iabcr, signalCurrent.i) annotation(
        Line(points = {{24.9, 2.1}, {44.9, 2.1}}, color = {0, 0, 255}, thickness = 0.5));
      connect(step4.y, add2.u2) annotation(
        Line(points = {{-77.6, 18}, {-68.6, 18}}, color = {0, 0, 127}));
      connect(cosineVoltage.plug_n, star2.plug_p) annotation(
        Line(points = {{-36, -14}, {-36, -16}}, color = {0, 0, 255}));
      connect(speedSensor.w, CONTROL_maq1.Wrm) annotation(
        Line(points = {{-10, 31.6}, {-10, 8.6}, {3, 8.6}}, color = {0, 0, 255}));
      connect(step1.y, add.u2) annotation(
        Line(points = {{-59.6, 38}, {-50.6, 38}}, color = {0, 0, 127}));
      connect(reactivePowerSensor1.reactivePower, rQs.Qs) annotation(
        Line(points = {{31, 48.6}, {31, 25}, {28.5, 25}}, color = {0, 0, 255}));
      connect(signalCurrent.plug_n, resistor1.plug_p) annotation(
        Line(points = {{52, 8}, {67, 8}}, color = {0, 0, 255}));
      connect(signalCurrent.plug_n, mit.plug_r) annotation(
        Line(points = {{52, 8}, {52, 32}}, color = {0, 0, 255}));
      connect(signalCurrent.plug_p, resistor1.plug_n) annotation(
        Line(points = {{52, -4}, {67, -4}}, color = {0, 0, 255}));
      connect(speedSensor.flange, mit.eixo) annotation(
        Line(points = {{-10, 40}, {37, 40}}));
      connect(ground3.p, star3.pin_n) annotation(
        Line(points = {{52, -30}, {52, -28}}, color = {0, 0, 255}));
      connect(step2.y, add1.u2) annotation(
        Line(points = {{95.6, 18}, {84.6, 18}}, color = {0, 0, 127}));
      connect(speedSensor.w, rv.W) annotation(
        Line(points = {{-10, 32}, {-10, 25}, {-0.5, 25}}, color = {0, 0, 127}));
      connect(add2.y, rv.Wref) annotation(
        Line(points = {{-60, 20}, {-29, 20}, {-29, 19.5}, {5, 19.5}}, color = {0, 0, 127}));
      connect(rQs.Qsref, add1.y) annotation(
        Line(points = {{23, 19.5}, {43, 19.5}, {43, 20}, {76, 20}}, color = {0, 0, 127}));
      connect(rQs.Qout, CONTROL_maq1.Qs_esp) annotation(
        Line(points = {{17.5, 25}, {17.5, 14}, {18, 14}}, color = {0, 0, 127}));
      connect(rv.Te, CONTROL_maq1.Te_esp) annotation(
        Line(points = {{10.5, 25}, {10.5, 14}, {10, 14}}, color = {0, 0, 127}));
    protected
      annotation(
        experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.000833333),
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})));
    end CONTROL_maqeDFIG_semzeros;

    model modelSimu
      extends Modelica.Icons.Example;
      import pi = Modelica.Constants.pi;
      Modelica.Electrical.Polyphase.Basic.Resistor R(R = fill(10, 3)) annotation(
        Placement(visible = true, transformation(origin = {87, -2}, extent = {{-6, 5}, {6, -5}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Basic.Star Y1 annotation(
        Placement(visible = true, transformation(origin = {-52, -26}, extent = {{6, -6}, {-6, 6}}, rotation = 90)));
      Modelica.Electrical.Polyphase.Sources.CosineVoltage Rede(V = fill(sqrt(2/3)*690, 3), f = fill(60, 3)) annotation(
        Placement(visible = true, transformation(origin = {-52, -12}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Basic.Star Y2 annotation(
        Placement(visible = true, transformation(origin = {72, -26}, extent = {{6, -6}, {-6, 6}}, rotation = 90)));
      AERO_model.DFIG_model.CONTROL_maq CONTROL_maq1 annotation(
        Placement(visible = true, transformation(origin = {14, -2}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Ground G1 annotation(
        Placement(visible = true, transformation(origin = {-52, -38}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      AERO_model.DFIG_model.MIT mit(D = 0) annotation(
        Placement(visible = true, transformation(origin = {68, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      AERO_model.DFIG_model.RQs rQs(ki_Q = 30) annotation(
        Placement(visible = true, transformation(origin = {23, 25}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
      AERO_model.DFIG_model.PLL pll1(zeta_PLL = 1) annotation(
        Placement(visible = true, transformation(origin = {-26, -6}, extent = {{-6, 6}, {6, -6}}, rotation = 0)));
      Modelica.Blocks.Sources.Step Qref(height = 0, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {40, 16}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
      Modelica.Blocks.Sources.Step Wref(height = 0, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {-12, 16}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.Torque torque annotation(
        Placement(visible = true, transformation(origin = {-28, 40}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sources.SignalCurrent FC annotation(
        Placement(visible = true, transformation(origin = {72, -2}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
      Modelica.Electrical.Analog.Basic.Ground G2 annotation(
        Placement(visible = true, transformation(origin = {72, -38}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor Wmed annotation(
        Placement(visible = true, transformation(origin = {-20, 36}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
      AERO_model.DFIG_model.RV rv(ts_Wrm = 1, zeta_Wrm = .5) annotation(
        Placement(visible = true, transformation(origin = {5, 25}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor Qmed annotation(
        Placement(visible = true, transformation(origin = {47, 49}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sensors.PowerSensor Pmed annotation(
        Placement(visible = true, transformation(origin = {-52, 2}, extent = {{4, -4}, {-4, 4}}, rotation = 90)));
      Modelica.Blocks.Sources.Step Tm(height = 0, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {-40, 40}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    equation
      connect(torque.flange, mit.eixo) annotation(
        Line(points = {{-24, 40}, {57, 40}}, color = {0, 0, 255}));
      connect(Y2.plug_p, FC.plug_p) annotation(
        Line(points = {{72, -20}, {72, -8}}, color = {0, 0, 255}));
      connect(pll1.WePll, CONTROL_maq1.We_PLL) annotation(
        Line(points = {{-19, -6}, {3.6, -6}}, color = {0, 0, 255}));
      connect(G1.p, Y1.pin_n) annotation(
        Line(points = {{-52, -34}, {-52, -32}}, color = {0, 0, 255}));
      connect(Rede.plug_p, pll1.vabc) annotation(
        Line(points = {{-52, -6}, {-33, -6}}, color = {0, 0, 255}));
      connect(CONTROL_maq1.iabcr, FC.i) annotation(
        Line(points = {{24.9, -1.9}, {34.9, -1.9}, {34.9, -2}, {65, -2}}, color = {0, 0, 255}, thickness = 0.5));
      connect(Rede.plug_n, Y1.plug_p) annotation(
        Line(points = {{-52, -18}, {-52, -20}}, color = {0, 0, 255}));
      connect(Wmed.w, CONTROL_maq1.Wrm) annotation(
        Line(points = {{-20, 32}, {-20, 5}, {3, 5}}, color = {0, 0, 255}));
      connect(FC.plug_n, R.plug_p) annotation(
        Line(points = {{72, 4}, {87, 4}}, color = {0, 0, 255}));
      connect(FC.plug_n, mit.plug_r) annotation(
        Line(points = {{72, 4}, {72, 32}}, color = {0, 0, 255}));
      connect(FC.plug_p, R.plug_n) annotation(
        Line(points = {{72, -8}, {87, -8}}, color = {0, 0, 255}));
      connect(Wmed.flange, mit.eixo) annotation(
        Line(points = {{-20, 40}, {57, 40}}));
      connect(G2.p, Y2.pin_n) annotation(
        Line(points = {{72, -34}, {72, -32}}, color = {0, 0, 255}));
      connect(Wmed.w, rv.W) annotation(
        Line(points = {{-20, 32}, {-20, 25}, {-0.5, 25}}, color = {0, 0, 255}));
      connect(rQs.Qout, CONTROL_maq1.Qs_esp) annotation(
        Line(points = {{17.5, 25}, {17.5, 9}, {18, 9}}, color = {0, 0, 255}));
      connect(rv.Te, CONTROL_maq1.Te_esp) annotation(
        Line(points = {{10.5, 25}, {10.5, 9}, {10, 9}}, color = {0, 0, 255}));
      connect(Qmed.reactivePower, rQs.Qs) annotation(
        Line(points = {{47, 45}, {47, 25}, {28.5, 25}}, color = {0, 0, 255}));
      connect(Qmed.plug_n, mit.plug_s) annotation(
        Line(points = {{51, 49}, {69, 49}}, color = {0, 0, 255}));
      connect(Pmed.nv, Rede.plug_n) annotation(
        Line(points = {{-48, 2}, {-62, 2}, {-62, -18}, {-52, -18}}, color = {0, 0, 255}));
      connect(Pmed.pv, Rede.plug_p) annotation(
        Line(points = {{-56, 2}, {-44, 2}, {-44, -6}, {-52, -6}}, color = {0, 0, 255}));
      connect(Pmed.pc, Qmed.plug_p) annotation(
        Line(points = {{-52, 6}, {-52, 49}, {43, 49}}, color = {0, 0, 255}));
      connect(Rede.plug_p, Pmed.nc) annotation(
        Line(points = {{-52, -6}, {-52, -2}}, color = {0, 0, 255}));
      connect(Wref.y, rv.Wref) annotation(
        Line(points = {{-8, 16}, {5, 16}, {5, 19.5}}, color = {0, 0, 255}));
      connect(Qref.y, rQs.Qsref) annotation(
        Line(points = {{36, 16}, {23, 16}, {23, 19.5}}, color = {0, 0, 255}));
      connect(Tm.y, torque.tau) annotation(
        Line(points = {{-36, 40}, {-32, 40}}, color = {0, 0, 127}));
      connect(pll1.VqPll, CONTROL_maq1.Vqs_PLL) annotation(
        Line(points = {{-19, -10}, {4, -10}}, color = {0, 0, 255}));
      connect(pll1.ThetaPll, CONTROL_maq1.thetaE_PLL) annotation(
        Line(points = {{-19, -2}, {4, -2}}, color = {0, 0, 255}));
    protected
      annotation(
        experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.000833333),
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(origin = {4, 25}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-8, 7}, {8, -7}}), Rectangle(origin = {24, 25}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-8, 7}, {8, -7}}), Rectangle(origin = {68, 40}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-12, 10}, {12, -10}}), Rectangle(origin = {-26, -6}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-10, 8}, {10, -8}}), Rectangle(origin = {14, -1}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-18, 13}, {18, -13}}), Rectangle(origin = {14, -16}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-18, 2}, {18, -2}}), Rectangle(origin = {-26, -16}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-10, 2}, {10, -2}}), Rectangle(origin = {68, 52}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-12, 2}, {12, -2}}), Rectangle(origin = {4, 34}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-8, 2}, {8, -2}}), Rectangle(origin = {24, 34}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-8, 2}, {8, -2}}), Text(origin = {68, 52}, textColor = {0, 0, 255}, extent = {{-10, 2}, {10, -2}}, textString = "MODELO MIT", textStyle = {TextStyle.Italic}), Text(origin = {14, -16}, textColor = {0, 0, 255}, extent = {{-16, 2}, {16, -2}}, textString = "MODELO EQ CORRENTE", textStyle = {TextStyle.Italic, TextStyle.Italic}), Text(origin = {-26, -16}, textColor = {0, 0, 255}, extent = {{-8, 2}, {8, -2}}, textString = "MODELO PLL", textStyle = {TextStyle.Italic}), Text(origin = {4, 34}, textColor = {0, 0, 255}, extent = {{-6, 2}, {6, -2}}, textString = "MODELO RV", textStyle = {TextStyle.Italic}), Text(origin = {24, 34}, textColor = {0, 0, 255}, extent = {{-6, 2}, {6, -2}}, textString = "MODELO RQ", textStyle = {TextStyle.Italic, TextStyle.Italic}), Text(extent = {{-22, 50}, {-22, 50}}, textString = "text"), Text(origin = {13, 42}, textColor = {85, 87, 83}, extent = {{-5, 2}, {5, -2}}, textString = "Tm, Wrm"), Text(origin = {-54, 22}, rotation = -90, textColor = {85, 87, 83}, extent = {{-8, 2}, {8, -2}}, textString = "vabcs, iabcs"), Text(origin = {74, 16}, rotation = -90, textColor = {85, 87, 83}, extent = {{-8, 2}, {8, -2}}, textString = "vabcr, iabcr"), Text(origin = {-23, 24}, rotation = -90, textColor = {85, 87, 83}, extent = {{-5, 2}, {5, -2}}, textString = "Wrm,med"), Text(origin = {6, 14}, rotation = 180, textColor = {85, 87, 83}, extent = {{-4, 2}, {4, -2}}, textString = "Te,reg"), Text(origin = {22, 14}, rotation = 180, textColor = {85, 87, 83}, extent = {{-4, 2}, {4, -2}}, textString = "Qs,reg"), Text(origin = {49, 31}, rotation = -90, textColor = {85, 87, 83}, extent = {{-4, 2}, {4, -2}}, textString = "Qs,med"), Text(origin = {-10, 0}, rotation = 180, textColor = {85, 87, 83}, extent = {{-4, 2}, {4, -2}}, textString = "Vq,PLL"), Text(origin = {-10, -4}, rotation = 180, textColor = {85, 87, 83}, extent = {{-4, 2}, {4, -2}}, textString = "We,PLL"), Text(origin = {-10, -8}, rotation = 180, textColor = {85, 87, 83}, extent = {{-4, 2}, {4, -2}}, textString = "Th,PLL"), Text(origin = {48, 0}, rotation = 180, textColor = {85, 87, 83}, extent = {{-4, 2}, {4, -2}}, textString = "iabcr")}));
    end modelSimu;

    model controlecorrente
      extends Modelica.Icons.Example;
      import pi = Modelica.Constants.pi;
      Modelica.Electrical.Polyphase.Sources.CosineVoltage Rede1(V = fill(sqrt(2/3)*690, 3), f = fill(60, 3)) annotation(
        Placement(visible = true, transformation(origin = {-92, -28}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Basic.Star Y11 annotation(
        Placement(visible = true, transformation(origin = {-92, -42}, extent = {{6, -6}, {-6, 6}}, rotation = 90)));
      Modelica.Electrical.Analog.Basic.Ground G11 annotation(
        Placement(visible = true, transformation(origin = {-92, -54}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput ThetaPll annotation(
        Placement(visible = true, transformation(origin = {-57, -51}, extent = {{-3, -3}, {3, 3}}, rotation = -90), iconTransformation(origin = {110, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.PI pi(T = kpPll/kiPll, initType = Modelica.Blocks.Types.Init.InitialOutput, k = -kpPll, y_start = We_ref) annotation(
        Placement(visible = true, transformation(origin = {-21, -37}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator integrator(initType = Modelica.Blocks.Types.Init.InitialOutput, y(fixed = true), y_start = 0) annotation(
        Placement(visible = true, transformation(origin = {-41, -37}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput WePll annotation(
        Placement(visible = true, transformation(origin = {-31, -51}, extent = {{-3, -3}, {3, 3}}, rotation = -90), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AERO_model.DFIG_model.tpark tpark annotation(
        Placement(visible = true, transformation(origin = {-57, -27}, extent = {{-5, 5}, {5, -5}}, rotation = 0)));
  Modelica.Electrical.Polyphase.Sensors.VoltageSensor Vmed annotation(
        Placement(visible = true, transformation(origin = {-79, -27}, extent = {{3, 3}, {-3, -3}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealOutput VqPll annotation(
        Placement(visible = true, transformation(origin = {-3, -27}, extent = {{-3, -3}, {3, 3}}, rotation = 0), iconTransformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Step Tm(height = 0, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {8, -54}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor Wmed annotation(
        Placement(visible = true, transformation(origin = {31, -57}, extent = {{-3, -3}, {3, 3}}, rotation = -90)));
  AERO_model.DFIG_model.MIT mit11(D = 0) annotation(
        Placement(visible = true, transformation(origin = {86, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sources.Torque torque annotation(
        Placement(visible = true, transformation(origin = {20, -54}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Sources.Step Wref(height = 0, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {8, -84}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Math.Add add(k1 = -1, k2 = +1) annotation(
        Placement(visible = true, transformation(origin = {40, -82}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Te annotation(
        Placement(visible = true, transformation(origin = {80, -80}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain(k = kp) annotation(
        Placement(visible = true, transformation(origin = {52, -68}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Math.Add add1(k1 = -1, k2 = +1) annotation(
        Placement(visible = true, transformation(origin = {66, -80}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator integrator1(initType = Modelica.Blocks.Types.Init.SteadyState, k = ki) annotation(
        Placement(visible = true, transformation(origin = {52, -82}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor Qmed annotation(
        Placement(visible = true, transformation(origin = {40, 94}, extent = {{-3, -3}, {3, 3}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground G1 annotation(
        Placement(visible = true, transformation(origin = {18, 62}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Electrical.Polyphase.Sources.CosineVoltage Rede(V = fill(sqrt(2/3)*690, 3), f = fill(60, 3)) annotation(
        Placement(visible = true, transformation(origin = {18, 88}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
  Modelica.Electrical.Polyphase.Basic.Star Y1 annotation(
        Placement(visible = true, transformation(origin = {18, 74}, extent = {{6, -6}, {-6, 6}}, rotation = 90)));
  AERO_model.DFIG_model.MIT mit(D = 0) annotation(
        Placement(visible = true, transformation(origin = {86, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add2(k1 = -1, k2 = +1) annotation(
        Placement(visible = true, transformation(origin = {52, 70}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator integrator2(initType = Modelica.Blocks.Types.Init.SteadyState, k = ki) annotation(
        Placement(visible = true, transformation(origin = {64, 70}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Qs annotation(
        Placement(visible = true, transformation(origin = {80, 70}, extent = {{-4, -4}, {4, 4}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Step Qref(height = 0, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {32, 68}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    equation
      connect(Rede1.plug_n, Y11.plug_p) annotation(
        Line(points = {{-92, -34}, {-92, -36}}, color = {0, 0, 255}));
      connect(integrator.y, tpark.theta) annotation(
        Line(points = {{-46.5, -37}, {-57, -37}, {-57, -32}}, color = {0, 0, 127}));
      connect(pi.y, integrator.u) annotation(
        Line(points = {{-26.5, -37}, {-35, -37}}, color = {0, 0, 127}));
      connect(Vmed.v, tpark.abc) annotation(
        Line(points = {{-75.7, -27}, {-61.7, -27}}, color = {0, 0, 127}, thickness = 0.5));
      connect(tpark.qd0[2], pi.u) annotation(
        Line(points = {{-52, -27}, {-15, -27}, {-15, -37}}, color = {0, 0, 127}));
      connect(integrator.y, ThetaPll) annotation(
        Line(points = {{-46.5, -37}, {-57, -37}, {-57, -51}}, color = {0, 0, 127}));
      connect(pi.y, WePll) annotation(
        Line(points = {{-26.5, -37}, {-31, -37}, {-31, -51}}, color = {0, 0, 127}));
      connect(tpark.qd0[1], VqPll) annotation(
        Line(points = {{-52, -27}, {-3, -27}}, color = {0, 0, 127}));
      connect(Y11.pin_n, G11.p) annotation(
        Line(points = {{-92, -48}, {-92, -50}}, color = {0, 0, 255}));
      connect(Rede1.plug_p, Vmed.plug_n) annotation(
        Line(points = {{-92, -22}, {-79, -22}, {-79, -24}}, color = {0, 0, 255}));
      connect(Rede1.plug_n, Vmed.plug_p) annotation(
        Line(points = {{-92, -34}, {-79, -34}, {-79, -30}}, color = {0, 0, 255}));
      connect(Wmed.flange, mit11.eixo) annotation(
        Line(points = {{31, -54}, {75, -54}}));
      connect(Tm.y, torque.tau) annotation(
        Line(points = {{12.4, -54}, {16.4, -54}}, color = {0, 0, 127}));
      connect(torque.flange, mit11.eixo) annotation(
        Line(points = {{24, -54}, {75, -54}}, color = {0, 0, 255}));
      connect(add.y, integrator1.u) annotation(
        Line(points = {{44.4, -82}, {47.4, -82}}, color = {0, 0, 127}));
      connect(gain.u, Wmed.w) annotation(
        Line(points = {{47.2, -68}, {31.4, -68}, {31.4, -60}}, color = {0, 0, 127}));
      connect(gain.y, add1.u1) annotation(
        Line(points = {{56.4, -68}, {61.4, -68}, {61.4, -78}}, color = {0, 0, 127}));
      connect(Wref.y, add.u2) annotation(
        Line(points = {{12.4, -84}, {35, -84}}, color = {0, 0, 127}));
      connect(integrator1.y, add1.u2) annotation(
        Line(points = {{56.4, -82}, {61.4, -82}}, color = {0, 0, 127}));
      connect(add1.y, Te) annotation(
        Line(points = {{70.4, -80}, {80.4, -80}}, color = {0, 0, 127}));
      connect(Wmed.w, add.u1) annotation(
        Line(points = {{31, -60.3}, {31, -80}, {35, -80}}, color = {0, 0, 127}));
      connect(Y1.pin_n, G1.p) annotation(
        Line(points = {{18, 68}, {18, 66}}, color = {0, 0, 255}));
      connect(Rede.plug_n, Y1.plug_p) annotation(
        Line(points = {{18, 82}, {18, 80}}, color = {0, 0, 255}));
  connect(Rede.plug_p, Qmed.plug_p) annotation(
        Line(points = {{18, 94}, {37, 94}}, color = {0, 0, 255}));
  connect(Qmed.plug_n, mit.plug_s) annotation(
        Line(points = {{43, 94}, {88, 94}}, color = {0, 0, 255}));
  connect(Qref.y, add2.u2) annotation(
        Line(points = {{36.4, 68}, {47.4, 68}}, color = {0, 0, 127}));
  connect(Qmed.reactivePower, add2.u1) annotation(
        Line(points = {{40, 91}, {40, 72}, {47, 72}}, color = {0, 0, 127}));
  connect(add2.y, integrator2.u) annotation(
        Line(points = {{56.4, 70}, {60.4, 70}}, color = {0, 0, 127}));
  connect(integrator2.y, Qs) annotation(
        Line(points = {{68.4, 70}, {80.4, 70}}, color = {0, 0, 127}));
    protected
      annotation(
        experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.000833333),
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(origin = {-37, -34}, rotation = -90, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-13, 30}, {13, -30}}), Text(extent = {{-22, 50}, {-22, 50}}, textString = "text"), Rectangle(origin = {51, -76}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-23, 14}, {23, -14}}), Rectangle(origin = {58, 69}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-14, 9}, {14, -9}})}));
    end controlecorrente;
    
    model modelSimu2
      extends Modelica.Icons.Example;
      import pi = Modelica.Constants.pi;
      Modelica.Electrical.Polyphase.Basic.Resistor R(R = fill(10, 3)) annotation(
        Placement(visible = true, transformation(origin = {87, -2}, extent = {{-6, 5}, {6, -5}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Basic.Star Y1 annotation(
        Placement(visible = true, transformation(origin = {-52, -26}, extent = {{6, -6}, {-6, 6}}, rotation = 90)));
      Modelica.Electrical.Polyphase.Sources.CosineVoltage Rede(V = fill(sqrt(2/3)*690, 3), f = fill(60, 3)) annotation(
        Placement(visible = true, transformation(origin = {-52, -12}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Basic.Star Y2 annotation(
        Placement(visible = true, transformation(origin = {72, -26}, extent = {{6, -6}, {-6, 6}}, rotation = 90)));
      AERO_model.DFIG_model.CONTROL_maq CONTROL_maq1 annotation(
        Placement(visible = true, transformation(origin = {14, -2}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
      Modelica.Electrical.Analog.Basic.Ground G1 annotation(
        Placement(visible = true, transformation(origin = {-52, -38}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      AERO_model.DFIG_model.MIT mit(D = 0) annotation(
        Placement(visible = true, transformation(origin = {68, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      AERO_model.DFIG_model.RQs rQs(ki_Q = 30) annotation(
        Placement(visible = true, transformation(origin = {23, 25}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
      AERO_model.DFIG_model.PLL pll1(zeta_PLL = 1) annotation(
        Placement(visible = true, transformation(origin = {-26, -6}, extent = {{-6, 6}, {6, -6}}, rotation = 0)));
      Modelica.Blocks.Sources.Step Qref(height = 0, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {40, 16}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
      Modelica.Blocks.Sources.Step Wref(height = 0, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {-40, 16}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.Torque torque annotation(
        Placement(visible = true, transformation(origin = {-28, 40}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sources.SignalCurrent FC annotation(
        Placement(visible = true, transformation(origin = {72, -2}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
      Modelica.Electrical.Analog.Basic.Ground G2 annotation(
        Placement(visible = true, transformation(origin = {72, -38}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor Wmed annotation(
        Placement(visible = true, transformation(origin = {-12, 36}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
      AERO_model.DFIG_model.RV rv(ts_Wrm = 1, zeta_Wrm = .5) annotation(
        Placement(visible = true, transformation(origin = {5, 25}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor Qmed annotation(
        Placement(visible = true, transformation(origin = {47, 49}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sensors.PowerSensor Pmed annotation(
        Placement(visible = true, transformation(origin = {-52, 2}, extent = {{4, -4}, {-4, 4}}, rotation = 90)));
      Modelica.Blocks.Sources.Step Tm(height = 0, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {-40, 40}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    equation
      connect(torque.flange, mit.eixo) annotation(
        Line(points = {{-24, 40}, {57, 40}}, color = {0, 0, 255}));
      connect(Y2.plug_p, FC.plug_p) annotation(
        Line(points = {{72, -20}, {72, -8}}, color = {0, 0, 255}));
      connect(pll1.WePll, CONTROL_maq1.We_PLL) annotation(
        Line(points = {{-19, -6}, {3.6, -6}}, color = {0, 0, 255}));
      connect(G1.p, Y1.pin_n) annotation(
        Line(points = {{-52, -34}, {-52, -32}}, color = {0, 0, 255}));
      connect(Rede.plug_p, pll1.vabc) annotation(
        Line(points = {{-52, -6}, {-33, -6}}, color = {0, 0, 255}));
      connect(CONTROL_maq1.iabcr, FC.i) annotation(
        Line(points = {{24.9, -1.9}, {34.9, -1.9}, {34.9, -2}, {65, -2}}, color = {0, 0, 255}, thickness = 0.5));
      connect(Rede.plug_n, Y1.plug_p) annotation(
        Line(points = {{-52, -18}, {-52, -20}}, color = {0, 0, 255}));
      connect(Wmed.w, CONTROL_maq1.Wrm) annotation(
        Line(points = {{-12, 32}, {-12, 5}, {3, 5}}, color = {0, 0, 255}));
      connect(FC.plug_n, R.plug_p) annotation(
        Line(points = {{72, 4}, {87, 4}}, color = {0, 0, 255}));
      connect(FC.plug_n, mit.plug_r) annotation(
        Line(points = {{72, 4}, {72, 32}}, color = {0, 0, 255}));
      connect(FC.plug_p, R.plug_n) annotation(
        Line(points = {{72, -8}, {87, -8}}, color = {0, 0, 255}));
      connect(Wmed.flange, mit.eixo) annotation(
        Line(points = {{-12, 40}, {57, 40}}));
      connect(G2.p, Y2.pin_n) annotation(
        Line(points = {{72, -34}, {72, -32}}, color = {0, 0, 255}));
      connect(Wmed.w, rv.W) annotation(
        Line(points = {{-12, 32}, {-12, 25}, {-0.5, 25}}, color = {0, 0, 255}));
      connect(rQs.Qout, CONTROL_maq1.Qs_esp) annotation(
        Line(points = {{17.5, 25}, {17.5, 9}, {18, 9}}, color = {0, 0, 255}));
      connect(rv.Te, CONTROL_maq1.Te_esp) annotation(
        Line(points = {{10.5, 25}, {10.5, 9}, {10, 9}}, color = {0, 0, 255}));
      connect(Qmed.reactivePower, rQs.Qs) annotation(
        Line(points = {{47, 45}, {47, 25}, {28.5, 25}}, color = {0, 0, 255}));
      connect(Qmed.plug_n, mit.plug_s) annotation(
        Line(points = {{51, 49}, {69, 49}}, color = {0, 0, 255}));
      connect(Pmed.nv, Rede.plug_n) annotation(
        Line(points = {{-48, 2}, {-62, 2}, {-62, -18}, {-52, -18}}, color = {0, 0, 255}));
      connect(Pmed.pv, Rede.plug_p) annotation(
        Line(points = {{-56, 2}, {-44, 2}, {-44, -6}, {-52, -6}}, color = {0, 0, 255}));
      connect(Pmed.pc, Qmed.plug_p) annotation(
        Line(points = {{-52, 6}, {-52, 49}, {43, 49}}, color = {0, 0, 255}));
      connect(Rede.plug_p, Pmed.nc) annotation(
        Line(points = {{-52, -6}, {-52, -2}}, color = {0, 0, 255}));
      connect(Qref.y, rQs.Qsref) annotation(
        Line(points = {{36, 16}, {23, 16}, {23, 19.5}}, color = {0, 0, 255}));
      connect(Tm.y, torque.tau) annotation(
        Line(points = {{-36, 40}, {-32, 40}}, color = {0, 0, 127}));
      connect(pll1.VqPll, CONTROL_maq1.Vqs_PLL) annotation(
        Line(points = {{-19, -10}, {4, -10}}, color = {0, 0, 255}));
      connect(pll1.ThetaPll, CONTROL_maq1.thetaE_PLL) annotation(
        Line(points = {{-19, -2}, {4, -2}}, color = {0, 0, 255}));
  connect(Wref.y, rv.Wref) annotation(
        Line(points = {{-36, 16}, {5, 16}, {5, 19.5}}, color = {0, 0, 127}));
    protected
      annotation(
        experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.000833333),
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(origin = {-35, 28}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-13, 20}, {13, -20}}), Text(extent = {{-22, 50}, {-22, 50}}, textString = "text")}));
    end modelSimu2;
    
    model modelSimu3
      extends Modelica.Icons.Example;
      import pi = Modelica.Constants.pi;
      Modelica.Mechanics.Rotational.Sources.Torque Tm annotation(
        Placement(visible = true, transformation(origin = {-42, 66}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sources.Torque Te annotation(
        Placement(visible = true, transformation(origin = {20, 66}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Inertia Jm(J = J) annotation(
        Placement(visible = true, transformation(origin = {8, 66}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Inertia Jt(J = J) annotation(
        Placement(visible = true, transformation(origin = {-30, 66}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.SpringDamper eixo annotation(
        Placement(visible = true, transformation(origin = {-18, 66}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
    equation
  connect(Tm.flange, Jt.flange_a) annotation(
        Line(points = {{-38, 66}, {-34, 66}}));
  connect(Jt.flange_b, eixo.flange_a) annotation(
        Line(points = {{-26, 66}, {-22, 66}}));
  connect(eixo.flange_b, Jm.flange_a) annotation(
        Line(points = {{-14, 66}, {4, 66}}));
  connect(Jm.flange_b, Te.flange) annotation(
        Line(points = {{12, 66}, {16, 66}}));
    protected
      annotation(
        experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.000833333),
        Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(origin = {-30, 65}, lineColor = {0, 0, 255}, fillColor = {153, 193, 241}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-20, 7}, {20, -7}}), Text(extent = {{-22, 50}, {-22, 50}}, textString = "text")}));
    end modelSimu3;
  end Examples;
  annotation(
    uses(Modelica(version = "4.0.0")));
end AERO_model;