package AERO_model

  model WT_DFIG
  parameter Interfaces.DWTData smData;
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor tacometro annotation(
      Placement(visible = true, transformation(origin = {14, 34}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
  DFIG_model.RV rv(Dtotal = smData.Dm + smData.Dtur, Jtotal = smData.Jm + smData.Jtur, Md_Wrm = smData.Md_Wrm, Td_Wrm = smData.Td_Wrm, ts_Wrm = smData.ts_Wrm, zeta_Wrm = smData.zeta_Wrm) annotation(
      Placement(visible = true, transformation(origin = {29, 15}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  TURBINA_model.TURBINA turbina(Dtur = smData.Dtur, Jtur = smData.Jtur, Md_beta = smData.Md_beta, N = smData.N, R = smData.R, Td_beta = smData.Td_beta, par = smData.p_ar, ts = smData.ts_beta, zeta = smData.zeta_beta)  annotation(
      Placement(visible = true, transformation(origin = {-32, 38}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
  Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor reactivePowerSensor annotation(
      Placement(visible = true, transformation(origin = {55, 51}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  TURBINA_model.EIXO eixo(Dtm = smData.Dtm, Ktm = smData.Ktm)  annotation(
      Placement(visible = true, transformation(origin = {0, 38}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
  Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = fill(smData.Rpf, 3)) annotation(
      Placement(visible = true, transformation(origin = {99, -14}, extent = {{-6, 5}, {6, -5}}, rotation = -90)));
  Modelica.Electrical.Polyphase.Basic.Star star annotation(
      Placement(visible = true, transformation(origin = {84, -38}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
  Modelica.Electrical.Polyphase.Sources.SignalCurrent FonteCorrente annotation(
      Placement(visible = true, transformation(origin = {84, -14}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
  TURBINA_model.CRTL_TUR crtl_tur(LBD_opt = smData.LBD_opt, N = smData.N, R = smData.R, Vw_max = smData.Vw_max, Vw_min = smData.Vw_min, Vw_nom = smData.Vw_nom, Vw_wmax = smData.Vw_wmax, Vw_wmin = smData.Vw_wmin)  annotation(
      Placement(visible = true, transformation(origin = {-32, 2}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
  DFIG_model.MIT mit(D = smData.Dm, J = smData.Jm, Llr = smData.Llr, Lls = smData.Lls, Lm = smData.Lm, Polos = smData.Polos, rr = smData.rr, rs = smData.rs)  annotation(
      Placement(visible = true, transformation(origin = {80, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  DFIG_model.RQs rQs(ki_Q = smData.ki_Q) annotation(
      Placement(visible = true, transformation(origin = {47, 15}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
  DFIG_model.CONTROL control(Lls = smData.Lls, Lm = smData.Lm, Polos = smData.Polos)  annotation(
      Placement(visible = true, transformation(origin = {38, -14}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation(
      Placement(visible = true, transformation(origin = {84, -50}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  DFIG_model.PLL pll annotation(
      Placement(visible = true, transformation(origin = {6, -18}, extent = {{-6, 6}, {6, -6}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput Vw annotation(
      Placement(visible = true, transformation(origin = {-110, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput Qesp annotation(
      Placement(visible = true, transformation(origin = {112, 0}, extent = {{-10, 10}, {10, -10}}, rotation = 180), iconTransformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Electrical.Polyphase.Interfaces.PositivePlug Vabcs annotation(
      Placement(visible = true, transformation(origin = {-104, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(tacometro.w, rv.W) annotation(
      Line(points = {{14, 29.6}, {14, 15}, {23.5, 15}}, color = {0, 0, 127}));
    connect(control.iabcr, FonteCorrente.i) annotation(
      Line(points = {{48.9, -13.9}, {58.9, -13.9}, {58.9, -14}, {77, -14}}, color = {0, 0, 127}, thickness = 0.5));
    connect(pll.VqPll, control.Vqs_PLL) annotation(
      Line(points = {{12.6, -21.6}, {28.6, -21.6}}, color = {0, 0, 127}));
    connect(star.plug_p, FonteCorrente.plug_p) annotation(
      Line(points = {{84, -32}, {84, -20}}, color = {0, 0, 255}));
    connect(eixo.flange_b, mit.eixo) annotation(
      Line(points = {{8.8, 38}, {69, 38}}));
    connect(tacometro.w, control.Wrm) annotation(
      Line(points = {{14, 29.6}, {14, -7.4}, {27, -7.4}}, color = {0, 0, 127}));
    connect(crtl_tur.beta, turbina.beta) annotation(
      Line(points = {{-32, 10.8}, {-32, 27.8}}, color = {0, 0, 127}));
    connect(rv.Te, control.Te_esp) annotation(
      Line(points = {{34.5, 15}, {34.5, 5.5}, {34.3, 5.5}, {34.3, -4}}, color = {0, 0, 127}));
    connect(ground.p, star.pin_n) annotation(
      Line(points = {{84, -46}, {84, -44}}, color = {0, 0, 255}));
    connect(rQs.Qout, control.Qs_esp) annotation(
      Line(points = {{41.5, 15}, {41.5, 5.5}, {41.7, 5.5}, {41.7, -4}}, color = {0, 0, 127}));
    connect(reactivePowerSensor.reactivePower, rQs.Qs) annotation(
      Line(points = {{55, 46.6}, {55, 15}, {52.5, 15}}, color = {0, 0, 127}));
    connect(pll.WePll, control.We_PLL) annotation(
      Line(points = {{12.6, -18}, {28.6, -18}}, color = {0, 0, 127}));
    connect(tacometro.flange, mit.eixo) annotation(
      Line(points = {{14, 38}, {69, 38}}));
    connect(FonteCorrente.plug_n, mit.plug_r) annotation(
      Line(points = {{84, -8}, {84, 30}}, color = {0, 0, 255}));
    connect(FonteCorrente.plug_p, resistor.plug_n) annotation(
      Line(points = {{84, -20}, {99, -20}}, color = {0, 0, 255}));
    connect(pll.ThetaPll, control.thetaE_PLL) annotation(
      Line(points = {{12.6, -14.4}, {28.6, -14.4}}, color = {0, 0, 127}));
    connect(crtl_tur.Wrm_opt, rv.Wref) annotation(
      Line(points = {{-23.2, 2}, {29, 2}, {29, 9.5}}, color = {0, 0, 127}));
    connect(FonteCorrente.plug_n, resistor.plug_p) annotation(
      Line(points = {{84, -8}, {99, -8}}, color = {0, 0, 255}));
    connect(eixo.flange_a, turbina.flange_Eixo) annotation(
      Line(points = {{-8, 38}, {-24, 38}}));
    connect(reactivePowerSensor.plug_n, mit.plug_s) annotation(
      Line(points = {{59, 51}, {81, 51}, {81, 47}}, color = {0, 0, 255}));
    connect(Vw, turbina.Vw) annotation(
      Line(points = {{-110, 38}, {-40, 38}}, color = {0, 0, 127}));
    connect(crtl_tur.Vw, Vw) annotation(
      Line(points = {{-40, 2}, {-66, 2}, {-66, 38}, {-110, 38}}, color = {0, 0, 255}));
    connect(Qesp, rQs.Qsref) annotation(
      Line(points = {{112, 0}, {48, 0}, {48, 10}}, color = {0, 0, 127}));
  connect(Vabcs, pll.vabc) annotation(
      Line(points = {{-104, -18}, {0, -18}}, color = {0, 0, 255}));
  connect(reactivePowerSensor.plug_p, Vabcs) annotation(
      Line(points = {{52, 52}, {-80, 52}, {-80, -18}, {-104, -18}}, color = {0, 0, 255}));
  annotation(
      Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Text( lineColor = {0, 0, 255}, extent = {{-80, 100}, {80, -100}}, textString = "WIND TURBINE
DFIG")}));end WT_DFIG;

  package Interfaces
    extends Modelica.Icons.InterfacesPackage;

    record DWTData
      extends Modelica.Icons.Record;
      import SI = Modelica.Units.SI;
      import pi = Modelica.Constants.pi;
     
      // Elec data:
      parameter SI.Resistance Rpf = 10 "Current source resistance" annotation(
        Dialog(group = "Eletrical Data"));
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
    
      // Mech data:
      parameter Integer Polos = 4 "Number of poles" annotation(
        Dialog(group = "Mechanical Data"));
      parameter SI.MomentOfInertia Jm = 59 "Machine moment of inertia" annotation(
        Dialog(group = "Mechanical Data"));
      parameter SI.RotationalDampingConstant Dm = 0 "Viscous friction of the machine" annotation(
        Dialog(group = "Mechanical Data"));
      parameter SI.MomentOfInertia Jtur = 474.5721 "Turbine moment of inertia" annotation(
        Dialog(group = "Mechanical Data")); 
      parameter SI.RotationalDampingConstant Dtur = 0.3925 "Viscous turbine friction" annotation(
        Dialog(group = "Mechanical Data"));
      parameter SI.RotationalSpringConstant Ktm = 6366.5064 "Shaft elasticity" annotation(
        Dialog(group = "Mechanical Data"));
      parameter SI.RotationalDampingConstant Dtm = 84.4577 "Viscous shaft friction" annotation(
        Dialog(group = "Mechanical Data"));
      
      // Aero data:
      parameter Real N = 111.5 "Gear ratio" annotation(
        Dialog(group = "Turbine Data"));
      parameter Real R = 37.5 "Helix radius" annotation(
        Dialog(group = "Turbine Data"));
      parameter Real p_ar = 1.225 "Air density" annotation(
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
      parameter Real LBD_opt = 6.3279 "Tip speed ratio optimal" annotation(
        Dialog(group = "Turbine Data"));
      
      // CTRL pitch data:
      parameter Real Md_beta = 5 "Max limiter of SlewRate" annotation(
        Dialog(group = "Control Data"));
      parameter SI.Time Td_beta = 1 "Constant time of SlewRate" annotation(
        Dialog(group = "Control Data"));
      parameter SI.Time ts_beta = 1e-2 "Settiling time by pitch angle" annotation(
        Dialog(group = "Pitch Control Data"));
      parameter Real zeta_beta = 0.7 "Damping constant by pitch angle" annotation(
        Dialog(group = "Pitch Control Data"));
      
      // CTRL Wrm data:
      parameter SI.Time ts_Wrm = 5 "Settiling time by speed" annotation(
        Dialog(group = "Velocity Control Data"));
      parameter Real zeta_Wrm = 0.7 "Damping constant by speed" annotation(
        Dialog(group = "Velocity Control Data"));
      parameter Real Md_Wrm = 0.1 "Max limiter SlewRate" annotation(
        Dialog(group = "Velocity Control Data"));
      parameter SI.Time Td_Wrm = 5 "Time constant SlewRate" annotation(
        Dialog(group = "Velocity Control Data"));
        
      // CTRL Qs data:
      parameter Real ki_Q = 30 "Integral constant by reactivepower" annotation(
        Dialog(group = "Reactive Power Control Data"));
        
      // CTRL PLL:
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

      annotation(
        defaultComponentName = "smData",
        defaultVariability = "Parameter");
    end DWTData;
  end Interfaces;
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

    model testeVento
      WINDSPEED windspeed(Vmax_rampa = 1, Vw_constante = 7, tf_rampa = 500, ti_rampa = 200)  annotation(
        Placement(visible = true, transformation(origin = {-12, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation

      annotation(
        experiment(StartTime = 0, StopTime = 1000, Tolerance = 1e-06, Interval = 0.01));
    end testeVento;
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
       
    
      Modelica.Blocks.Tables.CombiTable1Ds combiTable1Ds(extrapolation = Modelica.Blocks.Types.Extrapolation.LastTwoPoints, fileName = "/media/uemura/9E8CDF078CDED8BB1/MYGIT/Modelica/DFIG/mybeta.mat", tableName = "beta", tableOnFile = true, verboseRead = false) annotation(
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
      parameter SI.Time ts_Wrm = 5 "Settiling time by speed" annotation(
        Dialog(group = "Velocity Control Data"));
      parameter Real zeta_Wrm = 0.7 "Damping constant by speed" annotation(
        Dialog(group = "Velocity Control Data"));
      parameter Real Md_Wrm = 0.1 "Max limiter SlewRate" annotation(
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
      Modelica.Blocks.Nonlinear.SlewRateLimiter slewRateLimiter(Rising = Md_Wrm, Td = Td_Wrm) annotation(
        Placement(visible = true, transformation(origin = {-48, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(W, add.u2) annotation(
        Line(points = {{-92, -20}, {-28, -20}, {-28, -6}, {-18, -6}}, color = {0, 0, 127}));
      connect(add.y, PI.u) annotation(
        Line(points = {{6, 0}, {26, 0}}, color = {0, 0, 127}));
      connect(PI.y, Te) annotation(
        Line(points = {{49, 0}, {110, 0}}, color = {0, 0, 127}));
      connect(Wref, slewRateLimiter.u) annotation(
        Line(points = {{-92, 20}, {-60, 20}}, color = {0, 0, 127}));
      connect(slewRateLimiter.y, add.u1) annotation(
        Line(points = {{-36, 20}, {-28, 20}, {-28, 6}, {-18, 6}}, color = {0, 0, 127}));
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

    model testePll
      import pi = Modelica.Constants.pi;
      Modelica.Electrical.Polyphase.Sources.SignalVoltage signalVoltage annotation(
        Placement(visible = true, transformation(origin = {-40, 16}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Basic.Star star annotation(
        Placement(visible = true, transformation(origin = {-40, -16}, extent = {{-8, -8}, {8, 8}}, rotation = -90)));
      Modelica.Electrical.Analog.Basic.Ground ground annotation(
        Placement(visible = true, transformation(origin = {-40, -34}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
      PLL pll(Wn = 1e4, zeta = 0.95) annotation(
        Placement(visible = true, transformation(origin = {-12, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    algorithm
      if time <= 0.5 then
        signalVoltage.v[1] := 690 * sqrt(2 / 3) * cos(120 * pi * time);
        signalVoltage.v[2] := 690 * sqrt(2 / 3) * cos(120 * pi * time - 2 * pi / 3);
        signalVoltage.v[3] := 690 * sqrt(2 / 3) * cos(120 * pi * time + 2 * pi / 3);
      else
        signalVoltage.v[1] := 690 * sqrt(2 / 3) * cos(0.5 * 120 * pi * time);
        signalVoltage.v[2] := 690 * sqrt(2 / 3) * cos(0.5 * 120 * pi * time - 2 * pi / 3);
        signalVoltage.v[3] := 690 * sqrt(2 / 3) * cos(0.5 * 120 * pi * time + 2 * pi / 3);
      end if;
    equation
      connect(ground.p, star.pin_n) annotation(
        Line(points = {{-40, -28}, {-40, -24}}, color = {0, 0, 255}));
      connect(star.plug_p, signalVoltage.plug_n) annotation(
        Line(points = {{-40, -8}, {-40, 6}}, color = {0, 0, 255}));
      connect(signalVoltage.plug_p, pll.vabc) annotation(
        Line(points = {{-40, 26}, {-40, 50}, {-22, 50}}, color = {0, 0, 255}));
      annotation(
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 1.00001e-05));
    end testePll;
  end DFIG_model;

  package Examples
    extends Modelica.Icons.ExamplesPackage;

    model testePartida
      extends Modelica.Icons.Example;
      Modelica.Electrical.Polyphase.Sources.CosineVoltage Rede(V = fill(sqrt(2 / 3) * 690, 3), f = fill(60, 3)) annotation(
        Placement(visible = true, transformation(origin = {29, 45}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Basic.Star Y annotation(
        Placement(visible = true, transformation(origin = {11, 11}, extent = {{-7, -7}, {7, 7}}, rotation = -90)));
      Modelica.Electrical.Analog.Basic.Ground gnd annotation(
        Placement(visible = true, transformation(origin = {11, -3}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
      AERO_model.DFIG_model.MIT mit(RP = false) annotation(
        Placement(visible = true, transformation(origin = {6, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sources.QuadraticSpeedDependentTorque quadraticSpeedDependentTorque(tau_nominal = -10650, w_nominal = 188) annotation(
        Placement(visible = true, transformation(origin = {-36, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(gnd.p, Y.pin_n) annotation(
        Line(points = {{11, 2}, {11, 4}}, color = {0, 0, 255}));
      connect(mit.plug_s, Rede.plug_p) annotation(
        Line(points = {{7, 41}, {7, 45}, {22, 45}}, color = {0, 0, 255}));
      connect(Y.plug_p, mit.plug_r) annotation(
        Line(points = {{11, 18}, {11, 24}}, color = {0, 0, 255}));
      connect(Y.plug_p, Rede.plug_n) annotation(
        Line(points = {{11, 18}, {36, 18}, {36, 45}}, color = {0, 0, 255}));
      connect(quadraticSpeedDependentTorque.flange, mit.eixo) annotation(
        Line(points = {{-26, 32}, {-4, 32}}));
      annotation(
        experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-06, Interval = 0.0001));
    end testePartida;

    model controleDFIG
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
  Modelica.Electrical.Polyphase.Sources.CosineVoltage cosineVoltage(V = fill(sqrt(2 / 3) * 690, 3), f = fill(60, 3)) annotation(
        Placement(visible = true, transformation(origin = {-36, -8}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
  AERO_model.DFIG_model.RV rv(Dtotal = 0, Jtotal = 59, ts_Wrm = 1, zeta_Wrm = 1) annotation(
        Placement(visible = true, transformation(origin = {5, 31}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Electrical.Polyphase.Basic.Star star3 annotation(
        Placement(visible = true, transformation(origin = {52, -22}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
  Modelica.Blocks.Sources.Step step(height = -10.5e3, startTime = 1) annotation(
        Placement(visible = true, transformation(origin = {-64, 52}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Sources.Step step3(height = 0, offset = 0) annotation(
        Placement(visible = true, transformation(origin = {84, 32}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
  AERO_model.DFIG_model.CONTROL control1 annotation(
        Placement(visible = true, transformation(origin = {14, 2}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground ground2 annotation(
        Placement(visible = true, transformation(origin = {-36, -34}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Blocks.Sources.Step step1(height = 10.5e3, offset = 0, startTime = 6) annotation(
        Placement(visible = true, transformation(origin = {-64, 38}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  AERO_model.DFIG_model.MIT mit(D = 0) annotation(
        Placement(visible = true, transformation(origin = {48, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add2 annotation(
        Placement(visible = true, transformation(origin = {-66, 20}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  AERO_model.DFIG_model.RQs rQs(ki_Q = 10)  annotation(
        Placement(visible = true, transformation(origin = {23, 31}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
  Modelica.Blocks.Math.Add add1 annotation(
        Placement(visible = true, transformation(origin = {64, 20}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
  AERO_model.DFIG_model.PLL pll1 annotation(
        Placement(visible = true, transformation(origin = {-18, -2}, extent = {{-6, 6}, {6, -6}}, rotation = 0)));
  Modelica.Blocks.Sources.Step step2(height = 0, offset = -0.5e6, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {84, 18}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
  Modelica.Blocks.Sources.Step step4(height = 0, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {-84, 18}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sources.Torque torque annotation(
        Placement(visible = true, transformation(origin = {-28, 40}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Electrical.Polyphase.Sources.SignalCurrent signalCurrent annotation(
        Placement(visible = true, transformation(origin = {52, 2}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
  Modelica.Blocks.Sources.Step step5(height = 0, offset = 190) annotation(
        Placement(visible = true, transformation(origin = {-84, 32}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground ground3 annotation(
        Placement(visible = true, transformation(origin = {52, -34}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation(
        Placement(visible = true, transformation(origin = {-10, 36}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
    equation
  connect(step3.y, add1.u1) annotation(
        Line(points = {{79.6, 32}, {75.6, 32}, {75.6, 22}, {67.6, 22}}, color = {0, 0, 127}));
    connect(pll1.VqPll, control1.Vqs_PLL) annotation(
        Line(points = {{-11.4, -5.6}, {3.6, -5.6}}, color = {0, 0, 255}));
    connect(step5.y, add2.u1) annotation(
        Line(points = {{-79.6, 32}, {-75.6, 32}, {-75.6, 22}, {-69.6, 22}}, color = {0, 0, 127}));
  connect(torque.flange, mit.eixo) annotation(
        Line(points = {{-24, 40}, {38, 40}}, color = {0, 0, 255}));
    connect(star3.plug_p, signalCurrent.plug_p) annotation(
        Line(points = {{52, -16}, {52, -4}}, color = {0, 0, 255}));
    connect(pll1.WePll, control1.We_PLL) annotation(
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
  connect(rv.Te, control1.Te_esp) annotation(
        Line(points = {{10.5, 31}, {10.5, 13}}, color = {0, 0, 255}));
    connect(pll1.ThetaPll, control1.thetaE_PLL) annotation(
        Line(points = {{-11.4, 1.6}, {3.6, 1.6}}, color = {0, 0, 255}));
    connect(control1.iabcr, signalCurrent.i) annotation(
        Line(points = {{24.9, 2.1}, {44.9, 2.1}}, color = {0, 0, 255}, thickness = 0.5));
    connect(step4.y, add2.u2) annotation(
        Line(points = {{-79.6, 18}, {-70.6, 18}}, color = {0, 0, 127}));
    connect(cosineVoltage.plug_n, star2.plug_p) annotation(
        Line(points = {{-36, -14}, {-36, -16}}, color = {0, 0, 255}));
    connect(speedSensor.w, control1.Wrm) annotation(
        Line(points = {{-10, 31.6}, {-10, 8.6}, {3, 8.6}}, color = {0, 0, 255}));
    connect(step1.y, add.u2) annotation(
        Line(points = {{-59.6, 38}, {-50.6, 38}}, color = {0, 0, 127}));
  connect(reactivePowerSensor1.reactivePower, rQs.Qs) annotation(
        Line(points = {{31, 48.6}, {31, 30.6}, {28.5, 30.6}}, color = {0, 0, 255}));
    connect(signalCurrent.plug_n, resistor1.plug_p) annotation(
        Line(points = {{52, 8}, {67, 8}}, color = {0, 0, 255}));
  connect(add2.y, rv.Wref) annotation(
        Line(points = {{-61.6, 20}, {6.4, 20}, {6.4, 26}}, color = {0, 0, 127}));
  connect(signalCurrent.plug_n, mit.plug_r) annotation(
        Line(points = {{52, 8}, {52, 32}}, color = {0, 0, 255}));
  connect(add1.y, rQs.Qsref) annotation(
        Line(points = {{60, 20}, {23, 20}, {23, 25.5}}, color = {0, 0, 127}));
    connect(signalCurrent.plug_p, resistor1.plug_n) annotation(
        Line(points = {{52, -4}, {67, -4}}, color = {0, 0, 255}));
  connect(speedSensor.flange, mit.eixo) annotation(
        Line(points = {{-10, 40}, {37, 40}}));
    connect(ground3.p, star3.pin_n) annotation(
        Line(points = {{52, -30}, {52, -28}}, color = {0, 0, 255}));
  connect(step2.y, add1.u2) annotation(
        Line(points = {{79.6, 18}, {68.6, 18}}, color = {0, 0, 127}));
  connect(speedSensor.w, rv.W) annotation(
        Line(points = {{-10, 31.6}, {-10, 31}, {0, 31}}, color = {0, 0, 255}));
  connect(rQs.Qout, control1.Qs_esp) annotation(
        Line(points = {{18, 32}, {18, 14}}, color = {0, 0, 127}));
    protected
      annotation(
        experiment(StartTime = 0, StopTime = 12, Tolerance = 1e-06, Interval = 0.001));
    end controleDFIG;

    model windturbineDFIG
      extends Modelica.Icons.Example;
      import pi = Modelica.Constants.pi;
      Modelica.Electrical.Polyphase.Basic.Star star1 annotation(
        Placement(visible = true, transformation(origin = {-80, -38}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
      Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
        Placement(visible = true, transformation(origin = {-80, -50}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sources.CosineVoltage rede(V = fill(sqrt(2 / 3) * 690, 3), f = fill(60, 3)) annotation(
        Placement(visible = true, transformation(origin = {-80, -24}, extent = {{-6, 6}, {6, -6}}, rotation = -90)));
      Modelica.Electrical.Analog.Basic.Ground ground annotation(
        Placement(visible = true, transformation(origin = {64, -50}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Basic.Star star annotation(
        Placement(visible = true, transformation(origin = {64, -38}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
      AERO_model.DFIG_model.RV rv(ki = 200, kp = 200) annotation(
        Placement(visible = true, transformation(origin = {9, 15}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Sensors.SpeedSensor tacometro annotation(
        Placement(visible = true, transformation(origin = {0, 34}, extent = {{-4, -4}, {4, 4}}, rotation = -90)));
      Modelica.Blocks.Sources.Step Qref(height = 0, offset = 0, startTime = 0) annotation(
        Placement(visible = true, transformation(origin = {44, 2}, extent = {{4, -4}, {-4, 4}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Sensors.ReactivePowerSensor reactivePowerSensor annotation(
        Placement(visible = true, transformation(origin = {35, 51}, extent = {{-4, -4}, {4, 4}}, rotation = 0)));
      AERO_model.DFIG_model.RQs rQs annotation(
        Placement(visible = true, transformation(origin = {27, 15}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
      AERO_model.DFIG_model.CONTROL control annotation(
        Placement(visible = true, transformation(origin = {18, -14}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
      AERO_model.DFIG_model.MIT mit(D = 0) annotation(
        Placement(visible = true, transformation(origin = {60, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      AERO_model.DFIG_model.PLL pll annotation(
        Placement(visible = true, transformation(origin = {-14, -18}, extent = {{-6, 6}, {6, -6}}, rotation = 0)));
      Modelica.Electrical.Polyphase.Basic.Resistor resistor(R = fill(10, 3)) annotation(
        Placement(visible = true, transformation(origin = {79, -14}, extent = {{-6, 5}, {6, -5}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Sources.SignalCurrent FonteCorrente annotation(
        Placement(visible = true, transformation(origin = {64, -14}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
      AERO_model.TURBINA_model.TURBINA turbina annotation(
        Placement(visible = true, transformation(origin = {-38, 38}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
      AERO_model.TURBINA_model.CRTL_TUR crtl_tur annotation(
        Placement(visible = true, transformation(origin = {-38, 2}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
      AERO_model.WIND_model.WINDSPEED windspeed(Vmax_rajada = 4, Vmax_rampa = 0, Vw_constante = 8, tf_rajada = 400, tf_rampa = 0, ti_rajada = 20, ti_rampa = 0)  annotation(
        Placement(visible = true, transformation(origin = {-68, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  AERO_model.TURBINA_model.EIXO Eixo annotation(
        Placement(visible = true, transformation(origin = {-14, 38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
  connect(ground1.p, star1.pin_n) annotation(
        Line(points = {{-80, -46}, {-80, -44}}, color = {0, 0, 255}));
  connect(ground.p, star.pin_n) annotation(
        Line(points = {{64, -46}, {64, -44}}, color = {0, 0, 255}));
  connect(rede.plug_n, star1.plug_p) annotation(
        Line(points = {{-80, -30}, {-80, -32}}, color = {0, 0, 255}));
  connect(tacometro.w, rv.W) annotation(
        Line(points = {{0, 29.6}, {0, 14.6}, {3.5, 14.6}}, color = {0, 0, 127}));
  connect(Qref.y, rQs.Qsref) annotation(
        Line(points = {{39.6, 2}, {27, 2}, {27, 9.5}}, color = {0, 0, 127}));
  connect(reactivePowerSensor.reactivePower, rQs.Qs) annotation(
        Line(points = {{35, 46.6}, {35, 15}, {32.5, 15}}, color = {0, 0, 127}));
  connect(reactivePowerSensor.plug_p, rede.plug_p) annotation(
        Line(points = {{31, 51}, {-80, 51}, {-80, -18}}, color = {0, 0, 255}));
  connect(tacometro.flange, mit.eixo) annotation(
        Line(points = {{0, 38}, {49, 38}}));
  connect(tacometro.w, control.Wrm) annotation(
        Line(points = {{0, 29.6}, {0, -7.4}, {7, -7.4}}, color = {0, 0, 127}));
  connect(reactivePowerSensor.plug_n, mit.plug_s) annotation(
        Line(points = {{39, 51}, {61, 51}, {61, 47}}, color = {0, 0, 255}));
  connect(rede.plug_p, pll.vabc) annotation(
        Line(points = {{-80, -18}, {-20, -18}}, color = {0, 0, 255}));
  connect(pll.WePll, control.We_PLL) annotation(
        Line(points = {{-7.4, -18}, {7, -18}}, color = {0, 0, 127}));
  connect(control.iabcr, FonteCorrente.i) annotation(
        Line(points = {{28.9, -13.9}, {56.9, -13.9}}, color = {0, 0, 127}, thickness = 0.5));
  connect(FonteCorrente.plug_n, resistor.plug_p) annotation(
        Line(points = {{64, -8}, {79, -8}}, color = {0, 0, 255}));
  connect(star.plug_p, FonteCorrente.plug_p) annotation(
        Line(points = {{64, -32}, {64, -20}}, color = {0, 0, 255}));
  connect(FonteCorrente.plug_p, resistor.plug_n) annotation(
        Line(points = {{64, -20}, {79, -20}}, color = {0, 0, 255}));
  connect(FonteCorrente.plug_n, mit.plug_r) annotation(
        Line(points = {{64, -8}, {64, 30}}, color = {0, 0, 255}));
  connect(crtl_tur.beta, turbina.beta) annotation(
        Line(points = {{-38, 10.8}, {-38, 27.8}}, color = {0, 0, 127}));
  connect(crtl_tur.Wrm_opt, rv.Wref) annotation(
        Line(points = {{-29.2, 2}, {8.8, 2}, {8.8, 9.5}}, color = {0, 0, 127}));
  connect(windspeed.Vw, turbina.Vw) annotation(
        Line(points = {{-56.8, 38}, {-46.8, 38}}, color = {0, 0, 127}));
  connect(windspeed.Vw, crtl_tur.Vw) annotation(
        Line(points = {{-56.8, 38}, {-52.6, 38}, {-52.6, 2}, {-46.8, 2}}, color = {0, 0, 127}));
  connect(turbina.flange_Eixo, Eixo.flange_a) annotation(
        Line(points = {{-29.2, 38}, {-23.4, 38}}));
  connect(Eixo.flange_b, mit.eixo) annotation(
        Line(points = {{-3, 38}, {50, 38}}));
  connect(turbina.flange_Eixo, Eixo.flange_a) annotation(
        Line(points = {{-24, 38}, {-14, 38}}));
  connect(Eixo.flange_b, mit.eixo) annotation(
        Line(points = {{8, 38}, {70, 38}}));
  connect(rv.Te, control.Te_esp) annotation(
        Line(points = {{14, 16}, {14, -2}}, color = {0, 0, 127}));
  connect(rQs.Qout, control.Qs_esp) annotation(
        Line(points = {{22, 16}, {22, -2}}, color = {0, 0, 127}));
  connect(pll.ThetaPll, control.thetaE_PLL) annotation(
        Line(points = {{-8, -14}, {8, -14}}, color = {0, 0, 127}));
  connect(pll.VqPll, control.Vqs_PLL) annotation(
        Line(points = {{-8, -22}, {8, -22}}, color = {0, 0, 127}));
    protected
      annotation(
        experiment(StartTime = 0, StopTime = 450, Tolerance = 1e-06, Interval = 0.01));
    end windturbineDFIG;
    
    model testeModel
      extends Modelica.Icons.Example;
      Modelica.Electrical.Polyphase.Sources.CosineVoltage Rede(V = fill(sqrt(2 / 3) * 690, 3), f = fill(60, 3)) annotation(
        Placement(visible = true, transformation(origin = {53, 17}, extent = {{-7, -7}, {7, 7}}, rotation = -90)));
      Modelica.Electrical.Polyphase.Basic.Star Y annotation(
        Placement(visible = true, transformation(origin = {53, -1}, extent = {{-7, -7}, {7, 7}}, rotation = -90)));
      Modelica.Electrical.Analog.Basic.Ground gnd annotation(
        Placement(visible = true, transformation(origin = {53, -15}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  AERO_model.WT_DFIG wt_dfig(smData = smData)  annotation(
        Placement(visible = true, transformation(origin = {0, 24}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  AERO_model.WIND_model.WINDSPEED windspeed(Vmax_rajada = 0, Vw_constante = 12, tf_rajada = 0, tf_rampa = 0, ti_rajada = 0, ti_rampa = 0)  annotation(
        Placement(visible = true, transformation(origin = {-64, 24}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0)  annotation(
        Placement(visible = true, transformation(origin = {-64, -16}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));
  parameter Interfaces.DWTData smData annotation(
        Placement(visible = true, transformation(origin = {0, 66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
  connect(gnd.p, Y.pin_n) annotation(
        Line(points = {{53, -10}, {53, -8}}, color = {0, 0, 255}));
  connect(Y.plug_p, Rede.plug_n) annotation(
        Line(points = {{53, 6}, {53, 10}}, color = {0, 0, 255}));
  connect(windspeed.Vw, wt_dfig.Vw) annotation(
        Line(points = {{-55.04, 24}, {-21.04, 24}}, color = {0, 0, 127}));
  connect(wt_dfig.Vabcs, Rede.plug_p) annotation(
        Line(points = {{22, 24}, {54, 24}}, color = {0, 0, 255}));
  connect(const.y, wt_dfig.Qesp) annotation(
        Line(points = {{-56, -16}, {0, -16}, {0, 2}}, color = {0, 0, 127}));
      annotation(
        experiment(StartTime = 0, StopTime = 300, Tolerance = 1e-06, Interval = 0.001));
    end testeModel;
  end Examples;
  annotation(
    uses(Modelica(version = "4.0.0")));
end AERO_model;