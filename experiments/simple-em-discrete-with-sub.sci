m=100;
n=10;
dt=1/(m*n);
maxSec=3
A=[
    0,  0,     0;
    -1/dt, 0,  0;
    -1/dt^2,-1/dt, 0
  ];
B=[1;
   1/dt;
   1/dt^2
  ];

function U = ingresso(amplitude, phase)
  T=[1:m*maxSec+1];
  P=sin((T-1)*2*%pi/250+phase)*amplitude; // Funzione di ingresso (sinusoidale con periodo 250*n e max=amplitude (deg) )

  for j=[1:m*maxSec]
    for k=[1:n]
      U((j-1)*n+k)=P(j)+(P(j+1)-P(j))*k/n; // No: Definire in modo che accelerazione sia costante
    end
  end

endfunction

function [X,U] = goals(amplitude, phase)
  U=ingresso(35, 0);

  X=[0,0,0]; // Stato: Angolo, Velocità Angolare, Accelerazione Angolare
  for i=[1:m*n*maxSec]
    X(i+1,1:3) = X(i,1:3)*A'+(B*U(i))';
  end
endfunction

[X,U]=goals(35, 0);
plot(X(1:size(X,1),1));
plot(X(1:size(X,1),2));

