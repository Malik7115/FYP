a = -10;

b = 0.02;


A = [0 1;
     0 a];
 
B = [0;
     b];
 
C = [1 0];

 
%K = acker(A, B, [-5 -5])
 
H = acker(A', C', [-5 -5])'

T = 1/30;
l = 0;

% Delayed System

sys_cont = ss(A, B, C, 0);
sys_disc = c2d(sys_cont, T);
Ad = sys_disc.A;
Bd = sys_disc.B;
Cd = sys_disc.C;
[n,m] = size(Ad);

Kd = acker(Ad, Bd, 0.6 * ones(1, n))
Kd = [2000 1000];

[n,m] = size(Ad);

if(l > 0)
    Ad = [Ad,       Bd, zeros(n, l-1)];
    
    for i=1:l-1
        row_i = zeros(1, n+l);
        row_i(n+1+i) = 1;
        Ad = [Ad;
            row_i];
    end
    
    Ad = [Ad;
        zeros(1, n+l)];
    
    Bd = [zeros(n+l-1, 1);
        1];
    
    Cd = [Cd, zeros(1,l)];
end

% Controller Design
%
%Kd = acker(Ad, Bd, exp([-5 -5]*T))
%Kd = acker(Ad, Bd, 0.96 * ones(1, n+l))


%Observer Design
% Disturbance 
Aobs = [ Ad,            Bd;
         zeros(1, n+l), 1];
 
Bobs = [Bd;
        0];
 
Cobs = [Cd, 0];

Hd1 = acker(Ad', C', exp([-10 -10]*T))'
Hd = acker(Aobs', Cobs', 0.4 * ones(1, n+l+1))'

create_drone_control_file



