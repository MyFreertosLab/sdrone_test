// Questo modello si riferisce ad un asse vincolato 
// con fulcro al centro.
// L'ingresso è l'angolo Teta desiderato
// Lo stato è dato da Angolo, Velocità angolare e Accelerazione angolare

A=[
   -1,0,0;
   -1,-1,0;
   -1,-1,-1
];
B=[1;1;1];

n=100;
T=[0:n-1];
IN=T./n;
Teta(1:n)=20;
//Teta=10*sin(T*(2*%pi/250));
X0=[0;0;0];

function P1 = FP1(i)
  P1 = expm(A.*IN(i));
endfunction

function P2 = FP2(i,j)
  P2 = expm(A.*(IN(i)-IN(j)))
endfunction

function P3 = FP3(i)
  P3 = FP1(i)*B
endfunction

function P4 = FP4(i,j)
  P4 = FP2(i,j)*B
endfunction

function P5 = FP5(mx) 
  P5 = [0;0;0];
  for j = 1:mx
    P5 = P5+FP4(mx,j)*Teta(j);
  end
endfunction
function X = X(mx)
  X=FP1(mx)*X0 + (IN(mx)/(2*mx))*(FP3(mx)*Teta(1)+B*Teta(mx)+2*FP5(mx));
endfunction

function S = XS()
  for i = 1:n
    XV=X(i);
    S(i,1)=XV(1);
    S(i,2)=XV(2);
    S(i,3)=XV(3);
  end
endfunction

function DS = DXS()
  for i = 1:n
    XV=A*X(i)+B*Teta(i);
    DS(i,1)=XV(1);
    DS(i,2)=XV(2);
    DS(i,3)=XV(3);
  end
endfunction

//function ST = XST()
//  for i = 1:10
//    XV=XS();
//    S(i)=XV(1);
//  end
//endfunction

XV=XS();
DXV=DXS();
plot(XV(1:n,1))

