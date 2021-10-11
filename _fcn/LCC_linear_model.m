function [Ad,Bd,Cd] = LCC_linear_model(n,CF_bool,Ts)
% =========================================================================
%               calculate the DT-LTI model of LCC
% n:        number of following HDVs
% CF_bool:  whether car-following or free-driving
% Ts:       sampling time
% =========================================================================


% Driver Model: OVM
alpha       = 0.6; 
beta        = 0.9;
s_st        = 5;
s_go        = 35;
v_max       = 30;

s_star      = 20;

alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
alpha2 = alpha+beta;
alpha3 = beta;


A1 = [0,-1;alpha1,-alpha2];
A2 = [0,1;0,alpha3];

A = zeros(2*n+2,2*n+2);
B = zeros(2*n+2,1);
C = zeros(n+1,2*n+2);

if CF_bool
    A(1:2,1:2) = A1;
else
    A(1:2,1:2) = [0,1;0,0];
end

for i=2:(n+1)
    A(2*i-1:2*i,2*i-1:2*i) = A1;
    A(2*i-1:2*i,2*i-3:2*i-2) = A2;
    C(i,2*i) = 1;
end
B(2) = 1;
C(1,2) = 1;

accurate_model_bool = 1;

if accurate_model_bool
    % accurate model
    [Ad,Bd] = c2d(A,B,Ts);
else
    Ad      = Ts*A + eye(2*n+2);
    Bd      = B*Ts;
end
Cd          = C;

end

