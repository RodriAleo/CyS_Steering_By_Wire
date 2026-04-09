clc; clear;

% Declarar variables simbólicas
syms s
syms Lm Rm Bm Ka Jm Ke Kt Ks Bcr Mcr rL rP Bf Kf Jf

% Matriz A simbólica
A = sym(zeros(7,7));

A(1,1) = -Rm/Lm;
A(1,3) = -Ke/Lm;

A(2,3) = 1;

A(3,1) = Kt/Jm;
A(3,2) = -Ks/Jm;
A(3,3) = -Bm/Jm;
A(3,4) = Ks/(rP * Jm);

A(4,5) = 1;

A(5,2) = Ks/(rP * Mcr);
A(5,4) = -(2 * Kf/rL^2 + Ks/rP^2) / Mcr;
A(5,5) = -Bcr/Mcr;
A(5,6) = 2 * Kf/(rL * Mcr);

A(6,7) = 1;

A(7,4) = Kf/(rL * Jf);
A(7,6) = -Kf/Jf;
A(7,7) = -Bf/Jf;

% Vector de entrada de control (solo Vm)
B = sym(zeros(7,1));
B(1) = 1/Lm;

% Salida: delta (6to estado)
C = sym(zeros(1,7));
C(6) = 1;

% D = 0 porque no hay acción directa
D = 0;

% Función de transferencia simbólica: G(s) = C*(sI - A)^(-1)*B + D
I = sym(eye(7));
G_s = simplify(C * (s*I - A)^-1 * B + D);

% Mostrar resultado
disp('Función de transferencia simbólica G(s) = delta(s) / Vm(s):'); %[output:6b21a530]
pretty(collect(G_s,s)) %[output:637cddc6]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline","rightPanelPercent":36.9}
%---
%[output:6b21a530]
%   data: {"dataType":"text","outputData":{"text":"Función de transferencia simbólica G(s) = delta(s) \/ Vm(s):\n","truncated":false}}
%---
%[output:637cddc6]
%   data: {"dataType":"text","outputData":{"text":"                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               Kf Ks Kt rL rP\n-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n                2   2   7                   2   2                  2   2                  2   2                  2   2   6                    2                 2                  2   2                  2   2                  2   2                  2   2                  2   2                  2   2                  2   2                  2   2                  2   2   5                    2                   2                 2                 2                   2                 2                  2   2                  2   2                  2   2                  2   2                  2   2                  2   2                  2   2                  2   2                  2   2                  2   2                  2   2                  2   2   4                    2                 2                   2                   2                 2                 2                   2                 2                   2                 2                  2   2                  2   2                  2   2                  2   2                  2   2                  2   2                  2   2                  2   2                  2   2                  2   2   3                    2                 2                   2                 2                   2                 2                   2                 2                  2   2                  2   2                  2   2                  2   2                  2   2   2                    2                 2                 2                  2   2\n(Jf Jm Lm Mcr rL  rP ) s  + (Bcr Jf Jm Lm rL  rP  + Bf Jm Lm Mcr rL  rP  + Bm Jf Lm Mcr rL  rP  + Jf Jm Mcr Rm rL  rP ) s  + (2 Jf Jm Kf Lm rP  + Jf Jm Ks Lm rL  + Bcr Bf Jm Lm rL  rP  + Bcr Bm Jf Lm rL  rP  + Bf Bm Lm Mcr rL  rP  + Bcr Jf Jm Rm rL  rP  + Bf Jm Mcr Rm rL  rP  + Bm Jf Mcr Rm rL  rP  + Jf Ke Kt Mcr rL  rP  + Jm Kf Lm Mcr rL  rP  + Jf Ks Lm Mcr rL  rP ) s  + (2 Bf Jm Kf Lm rP  + 2 Bm Jf Kf Lm rP  + Bf Jm Ks Lm rL  + Bm Jf Ks Lm rL  + 2 Jf Jm Kf Rm rP  + Jf Jm Ks Rm rL  + Bcr Bf Bm Lm rL  rP  + Bcr Bf Jm Rm rL  rP  + Bcr Bm Jf Rm rL  rP  + Bcr Jf Ke Kt rL  rP  + Bcr Jm Kf Lm rL  rP  + Bf Bm Mcr Rm rL  rP  + Bcr Jf Ks Lm rL  rP  + Bf Ke Kt Mcr rL  rP  + Bm Kf Lm Mcr rL  rP  + Bf Ks Lm Mcr rL  rP  + Jm Kf Mcr Rm rL  rP  + Jf Ks Mcr Rm rL  rP ) s  + (2 Bf Bm Kf Lm rP  + Bf Bm Ks Lm rL  + 2 Bf Jm Kf Rm rP  + 2 Bm Jf Kf Rm rP  + Bf Jm Ks Rm rL  + Bm Jf Ks Rm rL  + 2 Jf Ke Kf Kt rP  + Jf Ke Ks Kt rL  + 2 Jf Kf Ks Lm rP  + Jm Kf Ks Lm rL  + Bcr Bf Bm Rm rL  rP  + Bcr Bf Ke Kt rL  rP  + Bcr Bm Kf Lm rL  rP  + Bcr Bf Ks Lm rL  rP  + Bcr Jm Kf Rm rL  rP  + Bcr Jf Ks Rm rL  rP  + Bm Kf Mcr Rm rL  rP  + Bf Ks Mcr Rm rL  rP  + Ke Kf Kt Mcr rL  rP  + Kf Ks Lm Mcr rL  rP ) s  + (2 Bf Bm Kf Rm rP  + Bf Bm Ks Rm rL  + 2 Bf Ke Kf Kt rP  + Bf Ke Ks Kt rL  + 2 Bf Kf Ks Lm rP  + Bm Kf Ks Lm rL  + 2 Jf Kf Ks Rm rP  + Jm Kf Ks Rm rL  + Bcr Bm Kf Rm rL  rP  + Bcr Bf Ks Rm rL  rP  + Bcr Ke Kf Kt rL  rP  + Bcr Kf Ks Lm rL  rP  + Kf Ks Mcr Rm rL  rP ) s  + (2 Bf Kf Ks Rm rP  + Bm Kf Ks Rm rL  + Ke Kf Ks Kt rL  + Bcr Kf Ks Rm rL  rP ) s\n\n","truncated":false}}
%---
