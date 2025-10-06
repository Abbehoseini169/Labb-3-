%% variabler
pnr = 960403;
s = tf('s');

Lm = 2;
Rm = 21;
b = 1;
[J,umax] = lab3robot(pnr);
Ktau = 38;
Km = 1/2;
n = 1/20;


%% upg 1 - hitta G(s)

P = Lm*J*s^3 + (Lm*b+Rm*J)*s^2 + (Rm*b+Km*Ktau)*s;
Q = n*Ktau;

G = Q/P;

[Gm_G, Pm_G, Wcg_G, Wcp_G] = margin(G);

lab3robot(G,pnr)

%% upg 2 - hitta optimalt p-reg med M<=5% och så kort risetime som möjligt

F_klist = 4.5:0.01:6;

data = table([], [], [], 'VariableNames', {'K','Overshoot','Risetime'});


for i=1:length(F_klist)
    Go = G*F_klist(i);
    sys = Go/(1+Go);
    info = stepinfo(sys);
    if info.Overshoot <= 5
        save_val = table(F_klist(i), info.Overshoot, info.RiseTime, 'VariableNames', {'K','Overshoot','Risetime'});
        data = [data; save_val];
    end
end



%% upg2 - plottar setp response med bästa K-värde

[minRT, index] = min(data.Risetime);

Kopt = data.K(index);

Go = Kopt*G;

Gc = Go/(1+Go);
step(Gc)
grid on

%% upg2 - plotta nyquistkurvan

nyquist(Go)
grid on

%% upg 3 - beräkna cross over frequency, phase margin för Go och bandwith för Gc

[Gm, Pm, Wcg, Wcp] = margin(Go);

bw = bandwidth(Gc);      % 3 dB-bandbredd

disp("cross-over frequncy")
Wcg
disp("phase margin")
Pm
disp("bandwith")
bw

% använde mig av fprintf för att få snyggare utsktift
fprintf('PM = %.2f deg vid ωc = %.4g rad/s\n', Pm, Wcp);
fprintf('Sluten slinga bandbredd ≈ %.4g rad/s\n', bw);

bode(Go);
grid on

%% upg4 - bara frågor i latex

% Delfråga 4
% GM is the factor by which you can multiply the current loop gain (which is 1) before instability.

[GM, PM, wcg, wcp] = margin(G);
Kcrit = GM;               % absolute critical K
fprintf('GM = %.4g, so Kcrit = %.4g\n', GM, Kcrit);

% We do a sanity check via the -180° crossing:
[mag,phase] = bode(G, wcg);           
Kcrit2 = 1/(abs(mag));   %  1/|G(jw_180)|
fprintf('Kcrit (from |G| at omega_180) ≈ %.4f\n', Kcrit2);



%% upg5 - bara frågor i latex







%% upg 6 - Designa lead-lag trans func F(s) som uppfyller nedan krav:
% 1) Overshoot <5%
% 2) Gc x4 snabbare än upg 2
% 3) |u|<umax för alla t (umax från robot)
% 4) stat err<0.05 då reference sig. är unit ramp dvs r(t) = t, då t>=0;.....
% ...annars 0
% 5) vill inte förstärka mätstörning för mkt, dvs hög frekvens amp ska ej .....
% ...vara för stor
% 6) begränsa controll gain vid låga frekvenser pga möjliga problem med....
% ...icke-linjäriteter


% Svara med motivering av val för Td, Ti, beta och gamma.

% hint: 
% a) starta med att fixa controller som löser 1-3
% b) anpassa denna till 4

%%

% a) krav 2 - p-reg f = k ger snabbhet

Tr_target = minRT/4;
wc_target = 0.68*1.8/Tr_target;

G_wc = evalfr(G, 1i*wc_target);  

Kfast = 0.9*1/abs(G_wc);
GoFast = Kfast*G;

GcFast = GoFast/(1+GoFast);
infoFast = stepinfo(GcFast);
newRT = infoFast.RiseTime
newOT = infoFast.Overshoot

speedIncrease = minRT/newRT

bode(GoFast, G), grid on 
hold off
step(GcFast, Gc), grid on 
hold off


%% a) krav 1 - Flead borde ge korrekt dämpning

[Gm_GoFast, Pm_GoFast, Wcg_GoFast, Wcp_GoFast] = margin(GoFast)

PMdelta = 80 - Pm_GoFast + 5.7
PMdelta_rad = deg2rad(PMdelta);
beta = (1-sin(PMdelta_rad))/(1 + sin(PMdelta_rad))


Td = 1/(sqrt(beta)*Wcg_GoFast);

Flead = (Td*s+1)/(beta*Td*s+1);

GoFlead = Flead*GoFast;

GcFlead = GoFlead/(1+GoFlead);
infoFlead = stepinfo(GcFlead);
newRTlead = infoFlead.RiseTime
newOTlead = infoFlead.Overshoot

figure(1)
nyquist(GoFlead), grid on

figure(2)
bode(GoFlead, GoFast), grid on

figure(3)
step(GcFlead, GcFast), grid on 
%%

% a) krav 3 - anpassa här då u är step  respons

[y,t] = step(GcFlead);
r = t;
e = 1-y;

u = lsim(Kfast * Flead, e, t);
ut_max = max(abs(u))
if ut_max < umax
    disp("uppfyller krav 3")
else
    disp("uppfyller INTE krav 3")
end

figure
plot(t,u)
hold off


y_simlead = lsim(GcFlead, r, t);
figure
plot(t,y_simlead)
hold on
plot(t,r)
hold off

rampDeltalag = y_simlead - r;
max(abs(rampDeltalag))
figure
plot(t,rampDeltalag)



%% b) krav 4 - Flag fixar stationärt fel

[Gm_GoFlead, Pm_GoFlead, Wcg_GoFlead, Wcp_GoFlead] = margin(GoFlead);
e1 = 0.05;

Kv_0 = dcgain(GoFlead)


%%


Ti = 10/Wcg_GoFlead;
%gamma = ;
evGolead = 1/GoFlead*s
adj_e1 = evalfr(evGolead, 0)

Flag = (Ti*s + 1)/(Ti*s + gamma);

GoFleadlag = GoFlead*Flag;

GcFleadlag = GoFleadlag/(1+GoFleadlag);

infoFleadlag = stepinfo(GcFleadlag);
newRTleadlag = infoFleadlag.RiseTime
newOTleadlag = infoFleadlag.Overshoot


figure(3)
step(GcFleadlag, GcFlead), grid on 


hold off

y_simleadlag = lsim(GcFleadlag, r, t);
figure
plot(t,y_simleadlag)
hold on
plot(t,r)
hold off

rampDeltaleadlag = y_simleadlag - r;
max(abs(rampDeltaleadlag))
figure
plot(t,rampDeltaleadlag)


%% Uppgift 12
% Variabler
J   = 4;
Lm  = 2;
Rm  = 21;
b   = 1;
Ktau= 38;
Km  = 0.5;
n   = 1/20;

A = [ 0,    n,      0;
      0,  -b/J,  Ktau/J;
      0, -Km/Lm, -Rm/Lm ];

B = [0;0; 1/Lm];
C = [1 0 0];  
D = 0;

% Pol placement 
zeta = 0.74; 
wn = 0.9;
p1 = -zeta*wn + 1i*wn*sqrt(1-zeta^2);
p2 = conj(p1);
p3 = -8*wn;
L = place(A,B,[p1 p2 p3]);

% Feedforward L0 för enhets DC-förstärkning r->y
Acl = A - B*L;
Gdc = -C*(Acl\B); 
L0  = 1/Gdc;


sys_cl = ss(Acl, B*L0, C, 0);
info = stepinfo(sys_cl);

% Uniform time grid 
Tend = max(30, 10*info.RiseTime);     
N    = 4000;                       % Num of points     
t    = linspace(0, Tend, N).';         

% Unit step reference  
r = ones(N,1);
[y, ~] = step(sys_cl, t); 

x0 = zeros(size(A,1),1);
[~, ~, x] = lsim(sys_cl, r, t, x0);    

u = -(x * L.') + L0*r;


Pdes = [p1 p2 p3];
fprintf('Desired poles:\n'); disp(Pdes.');

% Closed-loop performance
info = stepinfo(sys_cl);
fprintf('Rise time  = %.4g s\n', info.RiseTime);
fprintf('Overshoot  = %.2f %%\n', info.Overshoot);


umax = 110;                        
umax_meas = max(abs(u));
fprintf('max|u(t)|  = %.3f (limit %g) -> %s\n', umax_meas, umax, ...
        tern(umax_meas<=umax,'PASS','FAIL'));

disp('State feedback L ='); 
disp(L);
disp('Feedforward  L0 ='); 
disp(L0);


function out = tern(c,a,b) 
    if c, out=a; 
    else, out=b; 
    end 
end

fprintf('max|u(t)| = %.2f\n', max(abs(u)));


figure; 
plot(t,y, t,r,'--'); 
grid on; 
legend('y','r'); 
title('Closed-loop step (uniform t)');

figure;
plot(t,u); 
grid on; 
title('Control signal u(t)');



