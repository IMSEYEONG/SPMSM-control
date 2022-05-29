clc,clear;

syms s f;

Rs = 4.8;
Lq = 0.065;
Pol = 4;
Lamb = 0.272;
Jm = 0.000180;
Bm = 0.0021;

F = [-Rs/Lq -(Pol*Lamb)/(2*Lq) 0; (3*Pol*Lamb)/(2*2*Jm) -Bm/Jm 0; 0 1 0];
G = [1/Lq ;0 ;0];
H = [0 0 1];
J = [0];

FG = F*G;
F2G = F^2 *G;
C = [G F*G F^2*G];
determinant  = det(C);
invC = inv(C);

tn = [0 0 1] * invC;
TIc = [tn*F*F ;tn*F ;tn];
Tc = inv(TIc);
Ac = TIc*F*Tc;
Bc = TIc*G;
Cc = H*Tc;
Dc = J;
% sysG = ss(Ac,Bc,Cc,Dc);
% sysG = ss(F,G,H,J);
% G = tf(sysG)

% [Msys,T] = canon(sysG,'modal');

% smf=[s+Rs/Lq (Pol*Lamb)/(2*Lq) 0; -(3*Pol*Lamb)/(2*2*Jm) s+Bm/Jm 0; 0 -1 s];
% ZERO= [smf -G ; H J];
% zero = det(ZERO)
% pole = det(smf)

p = [-2.5556+2.5356i -2.5556-2.5356i -25.556];
% p = [-87.91 + 216.69i;-87.91 - 216.69i;-116.46];
a_c = (f-p(1))*(f-p(2))*(f-p(3));
a_c = sym2poly(a_c);
a_cF = F^3 + a_c(2)*F^2 + a_c(3)*F + a_c(4)*eye(3,3);
%K = [0 0 1]*invC*a_cF 
K = acker(F,G,p);

% bode(ss(F-G*K,G,H,0))

inverse = inv([F G ; H J]);
N = inverse*[0;0;0;1];
Nx = [N(1);N(2) ; N(3)];
Nu = [N(4)];
N_bar = Nu + K*Nx;
r = 1;
%bode(ss(F-G*K,N_bar*G,H,0))

% [num,den] = ss2tf(F-G*K,G,H,J)
% sys_cl = tf(num,den);
% pzmap(sys_cl)
% eig(F-G*K)
% stepinfo(1*sys_cl)
s = tf('s');
GO = H*inv((s*eye(3,3)-F))*G;
numG = conv(6.974358974358990e+04,6.974358974358990e+04);
denG = conv([1 85.512820512820620 3.880205128205135e+04 -1.145651560981955e-09],...
            [-1 -85.512820512820620 -3.880205128205135e+04 1.145651560981955e-09]);
sysG = tf(numG,denG);
% rlocus(sysG)
% [rho,lqr_poles] = rlocfind(sysG) %r.l. 상에서 rho 선정

rho = 361.3685;
lqr_pole = [-87.91 + 216.69i;-87.91 - 216.69i;-116.46];

K_lqr = acker(F,G,lqr_pole);
[num_lqr,den_lqr] = ss2tf(F-G*K_lqr,G,H,J);
sys_lqr = tf(num_lqr,den_lqr);
N_lqr_bar = Nu + K_lqr*Nx;
% stepinfo(sys_lqr)
% bode(ss(F-G*K_lqr,N_lqr_bar*G,H,0))

O = [H; H*F; H*F^2];
O = obsv(F,H);
det(O);

e_p = 7*lqr_pole;
L = acker(F',H',e_p);
L = L';

% rlocus(sysG)
% [que,lqr_poles] = rlocfind(sysG)
que=3.303080104630956e+03;
% lqr_poles
lqr_pole = [-8.166372161386107e+01 + 2.117601266243954e+02i -8.166372161386107e+01 - 2.117601266243954e+02i -99.675704121173510 + 0.000000000000000i];
K_lqr = acker(F,G,lqr_pole);
N_lqr_bar = Nu + K_lqr*Nx;

e_p_lqr = 7*lqr_pole;
L_lqr = acker(F',H',e_p_lqr);
L_lqr = L_lqr';

% bode(ss(F-G*K_lqr-L_lqr*H,L_lqr,K_lqr,0))
lqr_pole = [-87.91 + 216.69i;-87.91 - 216.69i;-116.46];
FI = [0 H;zeros(3,1) F];
GI = [0; G];
K_temp_integ = acker(FI,GI,[lqr_pole(1) lqr_pole(2) lqr_pole(3) lqr_pole(3)])
K_integ = [K_temp_integ(2) K_temp_integ(3) K_temp_integ(4)];
K1_integ = K_temp_integ(1);

K_temp_integ_NSRL = acker(FI,GI,[p(1) p(2) p(3) p(3)])
K_integ_NSRL = [K_temp_integ_NSRL(2) K_temp_integ_NSRL(3) K_temp_integ_NSRL(4)];
K1_integ_NSRL = K_temp_integ_NSRL(1);

%bode(ss(F-G*K_lqr-L_lqr*H,L_lqr,K_lqr,0))

% Rs = 4.8*1.1;
% Lq = 0.065*1.1;
% Lamb = 0.272*1.1;
% Jm = 0.000180*1.1;
% Bm = 0.0021*1.1;

Rs = 4.8*0.9;
Lq = 0.065*0.9;
Lamb = 0.272*0.9;
Jm = 0.000180*0.9;
Bm = 0.0021*0.9;

