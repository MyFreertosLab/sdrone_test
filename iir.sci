hz=iir(3,'bp','ellip',[.13 .15],[.008 .003]);
[hzm,fr]=frmag(hz,256);
plot2d(fr',hzm')
xtitle('Discrete IIR filter: band pass  0.15 < fr < 0.25 ',' ',' ');
q=poly(0,'q');     //to express the result in terms of the delay operator q=z^-1
hzd=horner(hz,1/q)

for i=1:size(A,1) 
    R(i)=flts(A(i,2), hz);
end
plot(R);


function [M]=movav(Sp,m);
  for k=size(Sp,1):-1:m
    M(k,1)=mean(Sp(k-m+1:k));
  end
  M(1:m-1)=Sp(1:m-1)
endfunction
