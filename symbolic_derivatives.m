% Group: Rositani
% Authors:
% Rositani Marco 10937256     
% Tosto Giuseppe 10934977
% Chiereghin Guglielmo 10666842      

%% Symbolic derivatives
clc
clear
syms x1 x2 x3 x4 u
syms k Jl Bl m g l Jm Bm 
Sys= [x2  
    -(k/Jl)*x1-(Bl/Jl)*x2+(k/Jl)*x3-(m*g*l/Jl)*cos(x1)
    x4
    (k/Jm)*x1-(k/Jm)*x3-(Bm/Jm)*x4];
C=[1 0 0 0];
X=[x1 
   x2 
   x3 
   x4];
B=[0 0 0 1/Jm]';
DerLie=nLie(4,C*X, Sys, X);
v=DerLie(4) + u*Lie(DerLie(3), B, X);
v= latex(v);
Larc= expand(DerLie(4))
Den= expand(Lie(DerLie(3), B, X))

function res=Lie(In, Sys, X)
    res=0;   
    for i = 1:length(X)
        res= res + Sys(i)*diff(In,X(i));
    end
end

function res=nLie(n, In, Sys, X)
res = X;
for i = 1:n
    if i == 1 
        res(1) = Lie(In,Sys, X);
    else
        res(i)= Lie(res(i-1), Sys, X);
    end
end
end