model rlc
  // Inclusão de bibliotecas:
  import SI = Modelica.Units.SI;
  // Declaração de parãmetros:
  parameter SI.Resistance R = 15 "Resistência";
  parameter SI.Capacitance C = 100e-6 "Capacitância";
  parameter SI.Inductance L = 100e-3 "Indutãncia";
  // Definindo variáveis auxiliares:
  SI.Voltage v_S, v_L, v_C, v_R;
  SI.Current i;
initial equation
// Condições de inicialização:
der(i)=0;
der(v_C)=0;
equation
// Definindo a tensão de entrada:
  v_S = if time <= 0.25 then 1.0 else 0.0;
// Tensão sob os componentes:
  v_L = L*der(i);
  i = C*der(v_C);
  v_R = R*i;
// Lei de Kirchhoff:
  v_S = v_L + v_C + v_R;
  annotation(
    experiment(StartTime = 0, StopTime = 0.5, Tolerance = 1e-06, Interval = 0.0001));
end rlc;