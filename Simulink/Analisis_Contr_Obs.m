clc; clear;

% Símbolos
syms Lm Rm Bm Ka Jm Ke Kt Ks Bcr Mcr rL rP Bf Kf Jf

% Estado: x = [im; thetam; omegam; yc; vc; delta; psif]
% 7 estados, 1 entrada de control, 3 perturbaciones

% Matriz A (7x7)
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

% Matriz de entrada de control (Vm)
Bc = sym(zeros(7,1));
Bc(1) = 1/Lm;

% Matriz de perturbaciones [Ff; Tf; Ta]
Bd = sym(zeros(7,3));
Bd(5,1) = -1/Mcr;  % Ff: fuerza de fricción en la cremallera
Bd(7,2) = -1/Jf;        % Tf: torque de fricción en la rueda
Bd(7,3) = -1/Jf;        % Ta: torque de autoalineamiento

% Matriz de entrada completa
B = [Bc Bd];

% Estado: x = [i; theta; omega; Y; Vr; delta; psi]
% Matriz C
C = zeros(1,7);
C(1,6)=1;
%C(2,3)=1;
%%
%[text] CONTROLABILIDAD
Co = [Bc A*Bc A^2*Bc A^3*Bc A^4*Bc A^5*Bc A^6*Bc];

% Calcular el determinante (solo si Co es cuadrada, o tomar rango si no)
if size(Co,1) == size(Co,2) %[output:group:0813e14d]
    detCo = simplify(det(Co));
    disp('Determinante de la matriz de controlabilidad:'); %[output:53fd5e50]
    disp(detCo); %[output:4ed45bab]

    rango_Co = rank(Co) %[output:14c72869]
end %[output:group:0813e14d]

% Asignar valores numéricos
valores = {
    Kf, 26000;
    Ks, 3500;
    Kt, 0.35;
    Jf, 0.27;
    Jm, 1.9;
    Lm, 0.002;
    Mcr, 2;
    rL, 0.3;
    rP, 0.035
};

% Evaluar el determinante numéricamente
det_num = double(subs(detCo, ...
    [Kf Ks Kt Jf Jm Lm Mcr rL rP], ...
    [26000 3500 0.35 0.27 1.9 0.002 2 0.3 0.035]));

% Mostrar resultado
disp('Valor numérico del determinante de la matriz de controlabilidad:'); %[output:57406725]
disp(det_num); %[output:0db31616]

%%
%[text] OBSERVABILIDAD
% A y C deben estar definidos simbólicamente antes
% C observa solo la sexta variable (delta)
C = sym(zeros(1,7));
C(2) = 1 %[output:0ef2259c]

% Matriz de observabilidad Ob = [C; C*A; C*A^2; ... ; C*A^6]
Ob = C;
for i = 1:6
    Ob = [Ob; C*A^i];
end

detOb = simplify(det(Ob)) %[output:59329e04]
detOb_num = double(subs(detOb,... %[output:group:7b1cc10a] %[output:977657d4]
    [Kf Ks Kt Jf Jm Lm Mcr rL rP Rm Bf Bcr], ... %[output:977657d4]
    [26000 3500 0.35 0.27 1.9 0.002 2 0.3 0.035 0.6 30 1.032])) %[output:group:7b1cc10a] %[output:977657d4]

Ob_num = subs(Ob, ...
    [Kf Ks Kt Jf Jm Lm Mcr rL rP], ...
    [26000 3500 0.35 0.27 1.9 0.002 2 0.3 0.035]);

rango_Ob = rank(Ob_num);
disp(['Rango numérico de la matriz de observabilidad: ', num2str(rango_Ob)]); %[output:30decbc6]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":39}
%---
%[output:53fd5e50]
%   data: {"dataType":"text","outputData":{"text":"Determinante de la matriz de controlabilidad:\n","truncated":false}}
%---
%[output:4ed45bab]
%   data: {"dataType":"symbolic","outputData":{"name":"","value":"-\\frac{{\\mathrm{Kf}}^2 \\,{\\mathrm{Ks}}^4 \\,{\\mathrm{Kt}}^6 }{{\\mathrm{Jf}}^2 \\,{\\mathrm{Jm}}^6 \\,{\\mathrm{Lm}}^7 \\,{\\mathrm{Mcr}}^4 \\,{\\mathrm{rL}}^2 \\,{\\mathrm{rP}}^4 }"}}
%---
%[output:14c72869]
%   data: {"dataType":"not_yet_implemented_variable","outputData":{"columns":"1","name":"rango_Co","rows":"1","value":"7"},"version":0}
%---
%[output:57406725]
%   data: {"dataType":"text","outputData":{"text":"Valor numérico del determinante de la matriz de controlabilidad:\n","truncated":false}}
%---
%[output:0db31616]
%   data: {"dataType":"text","outputData":{"text":"  -1.9658e+44\n\n","truncated":false}}
%---
%[output:0ef2259c]
%   data: {"dataType":"symbolic","outputData":{"name":"C","value":"\\left(\\begin{array}{ccccccc}\n0 & 1 & 0 & 0 & 0 & 0 & 0\n\\end{array}\\right)"}}
%---
%[output:59329e04]
%   data: {"dataType":"symbolic","outputData":{"name":"detOb","value":"\\frac{4\\,{\\mathrm{Kf}}^2 \\,{\\mathrm{Ks}}^4 \\,\\mathrm{Kt}\\,{\\left(\\mathrm{Kf}\\,\\mathrm{Ks}\\,{\\mathrm{Lm}}^4 \\,{\\mathrm{rL}}^2 +2\\,\\mathrm{Jf}\\,\\mathrm{Kf}\\,{\\mathrm{Lm}}^2 \\,{\\mathrm{Rm}}^2 \\,{\\mathrm{rP}}^2 +\\mathrm{Jf}\\,\\mathrm{Ks}\\,{\\mathrm{Lm}}^2 \\,{\\mathrm{Rm}}^2 \\,{\\mathrm{rL}}^2 +\\mathrm{Jf}\\,\\mathrm{Mcr}\\,{\\mathrm{Rm}}^4 \\,{\\mathrm{rL}}^2 \\,{\\mathrm{rP}}^2 -2\\,\\mathrm{Bf}\\,\\mathrm{Kf}\\,{\\mathrm{Lm}}^3 \\,\\mathrm{Rm}\\,{\\mathrm{rP}}^2 -\\mathrm{Bf}\\,\\mathrm{Ks}\\,{\\mathrm{Lm}}^3 \\,\\mathrm{Rm}\\,{\\mathrm{rL}}^2 -\\mathrm{Bcr}\\,\\mathrm{Jf}\\,\\mathrm{Lm}\\,{\\mathrm{Rm}}^3 \\,{\\mathrm{rL}}^2 \\,{\\mathrm{rP}}^2 -\\mathrm{Bcr}\\,\\mathrm{Kf}\\,{\\mathrm{Lm}}^3 \\,\\mathrm{Rm}\\,{\\mathrm{rL}}^2 \\,{\\mathrm{rP}}^2 -\\mathrm{Bf}\\,\\mathrm{Lm}\\,\\mathrm{Mcr}\\,{\\mathrm{Rm}}^3 \\,{\\mathrm{rL}}^2 \\,{\\mathrm{rP}}^2 +\\mathrm{Bcr}\\,\\mathrm{Bf}\\,{\\mathrm{Lm}}^2 \\,{\\mathrm{Rm}}^2 \\,{\\mathrm{rL}}^2 \\,{\\mathrm{rP}}^2 +\\mathrm{Kf}\\,{\\mathrm{Lm}}^2 \\,\\mathrm{Mcr}\\,{\\mathrm{Rm}}^2 \\,{\\mathrm{rL}}^2 \\,{\\mathrm{rP}}^2 \\right)}}{\\mathrm{Jf}\\,{\\mathrm{Jm}}^5 \\,{\\mathrm{Lm}}^4 \\,{\\mathrm{Mcr}}^3 \\,{\\mathrm{rL}}^4 \\,{\\mathrm{rP}}^6 }"}}
%---
%[output:977657d4]
%   data: {"dataType":"not_yet_implemented_variable","outputData":{"columns":"1","name":"detOb_num","rows":"1","value":"2.6397e+39"},"version":0}
%---
%[output:30decbc6]
%   data: {"dataType":"text","outputData":{"text":"Rango numérico de la matriz de observabilidad: 7\n","truncated":false}}
%---
