m=100;
dt=1/(m);
maxSec=10
Thrust=2200;
b=0.5;

A=[
    0,  0,     0;
    -1/dt, 0,  0;
    -1/dt^2, -1/dt, 0
  ];
B=[1;
   1/dt;
   1/dt^2
  ];
C=[0,0,b/2;0,0,-b/2];
D=[0,1;0,1];

function U = ingresso(amplitude, phase)
  T=[1:m*maxSec+1];
  P=sin((T-1)*2*%pi/500+phase)*amplitude; // Funzione di ingresso (sinusoidale con periodo 500 e max=amplitude (deg) )

  U = P;
endfunction

function [X,U,Y] = goals(amplitude, phase)
  U=ingresso(35, 0);
  Y=[0,0];
  X=[0,0,0]; // Stato: Angolo, Velocità Angolare, Accelerazione Angolare
  for i=[1:m*maxSec]
    X(i+1,1:3) = X(i,1:3)*A'+(B*U(i))';
    Y(i,1:2) = X(i+1,1:3)*C'+(D*[U(i);Thrust])'
  end
endfunction

[X,U,Y]=goals(35, 0);
plot(X(1:size(X,1),1));
plot(X(1:size(X,1),2));

