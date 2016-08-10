function pz_mx = sensorFM(nr,dx,mu,sigma)
x = (1:nr)/dx;
% for k = 1:nr
%     pz_mx(k) = 1/sqrt(2*pi*sigma*2)*exp(-1/2*(dx*k-dt)^2/sigma^2);
% end
pz_mx = normpdf(x,mu,sigma);