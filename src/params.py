"""Parametros fisicos da junta do cotovelo do AuroraArm (bancada de ensaio).
Fonte: enunciado do trabalho, secao 2 (Onde?). Unidades SI."""
R = 0.8          # [ohm]  resistencia de armadura
L = 1.2e-3       # [H]    indutancia de armadura
KT = 0.085       # [N m/A]
KE = 0.085       # [V s/rad]
J = 4.2e-4       # [kg m^2] inercia refletida no eixo do motor
B = 1.5e-4       # [N m s/rad] atrito viscoso refletido
N = 100          # relacao de reducao (harmonic drive)
VMAX = 48.0      # [V]
IMAX = 8.0       # [A]
TS = 2e-3        # [s] periodo de amostragem do laco de posicao
# ponto de operacao critico (dado do enunciado)
W_SS = 450.0     # [rad/s] no eixo do motor
T_LOAD = 0.25    # [N m] torque de carga refletido (valor do enunciado)
# Seguranca colaborativa
V_TCP_MAX = 0.250   # [m/s] REQ-SAFE-AA-004
R_TCP = 0.35        # [m] raio do payload (enunciado)
D_ZONE = 0.5        # [m] REQ-SAFE-AA-001
T_DEADLINE = 0.100  # [s]
SENSOR_DELAY = 0.020  # [s]
JITTER_CYCLES = 1

V_TCP_DESIGN = 0.2339   # [m/s] CALIBRADO por src/calib_collab.py (pior caso de inversao de rampa: pico real <= 245 mm/s)
