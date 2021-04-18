// Implements: dX(i) = A*X(i)+B*U(i)
function [X,dX] = steps(t,n,U,X0,dX0,start)
  dt=1/n;

  X = X0;

  dX = dX0;

  for i=[1+start:t+start]

    dX(i+1,1) = (U(i)-X(i))/dt          // Omega: Velocità richiesta (Prediction)
    dX(i+1,2) = (dX(i+1,1)-dX(i,1))/dt; // Alfa: Accelerazione richiesta (Prediction)
    dX(i+1,3) = (dX(i+1,2)-dX(i,2))/dt; // dAlfa

    X(i+1,1) = dX(i+1,1)*dt+X(i,1); // Teta
    X(i+1,2) = dX(i+1,2)*dt+X(i,2); // Omega
    X(i+1,3) = dX(i+1,3)*dt+X(i,3); // Alfa

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

// Teta(i+1) = (U(i)-X(i)) + (U(i-1)-X(i-1)) + .... + (U(1)-X(1))  => Teta(i+1) = SUM(j=1:i, U(j)-X(j))
// Omega(i+1) = (Teta(i+1) - Teta(i))/dt = (SUM(j=1:i, U(j)-X(j)) - SUM(j=1:i-1, U(j)-X(j)))/dt = (U(i) - X(i))/dt
// Alfa(i+1) = (Omega(i+1) - Omega(i))/dt = (U(i) - X(i))/dt - (U(i-1) - X(i-1))/dt = ????? deve essere in funzione di U(i) e X(i). Inserire Omega nello stato X.

// Teta(i+1) = (U(i)-X(i,1)) + (U(i-1)-X(i-1,1)) + .... + (U(1)-X(1,1))  => Teta(i+1) = SUM(j=1:i, U(j)-X(j,1)) = U(i)-X(1,1)
// X(i+1,2) = Omega(i+1) = (Teta(i+1) - Teta(i))/dt = (SUM(j=1:i, U(j)-X(j,1)) - SUM(j=1:i-1, U(j)-X(j,1)))/dt = (U(i) - X(i,1))/dt
// X(i+1,3) = Alfa(i+1) = (Omega(i+1) - X(i,2))/dt = ((U(i) - X(i,1))/dt - X(i,2))/dt
// dAlfa(i+1) = (Alfa(i+1) - Alfa(i))/dt = (((U(i) - X(i,1))/dt - X(i,2))/dt - X(i,3))/dt

