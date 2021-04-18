function [X,dX] = steps(t,n,U,X0,dX0,start)
  dt=1/n;
  A=[
    -1/dt,  0,     0;
    -1/dt^2,-1/dt,  0;
    -1/dt^3,-1/dt^2,-1/dt
  ];
  B=[1/dt;
   1/dt^2;
   1/dt^3
  ];


  X = X0;
  dX = dX0;

  for i=[1+start:t+start]
    dX(i+1,1:3) = X(i,1:3)*A'+(B*U(i))'

    X(i+1,1) = X(i,1)+dX(i+1,1)*dt;
    X(i+1,2) = dX(i+1,1);
    X(i+1,3) = dX(i+1,2);
  end
endfunction

function [X,dX,P,U] = goals(amplitude, phase)
  m=100;
  T=[1:m+1];
  P=sin((T-1)*2*%pi/20+phase)*amplitude; // Funzione di ingresso (sinusoidale con periodo 20 e max=amplitude (deg) )

  // Stato iniziale e steps
  n=100;
  X=[0,0,0]; // Stato: Angolo, Velocità Angolare, Accelerazione Angolare
  dX=[0,0,0]; // Stato: Velocità Angolare, Accelerazione Angolare, Derivata Accelerazione Angolare

  for i=[1:m+1]
    U((i-1)*n+1:i*n)=X((i-1)*n+1) + (P(i)-X((i-1)*n+1))*([1:n]./n); // Funzione di ingresso (lineare a gradini)
    [X,dX]=steps(n,n,U,X,dX,(i-1)*n);
  end
endfunction

[X,dX,P,U]=goals(35, %pi/2);
plot(X(1:size(X,1),1));

