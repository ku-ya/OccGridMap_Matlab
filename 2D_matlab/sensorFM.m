function pz_mx = sensorFM(nr,dx,mu,param)
sigma = param.sigma;
x = (1:nr)*mu/nr;
% for k = 1:nr
%     pz_mx(k) = 1/sqrt(2*pi*sigma*2)*exp(-1/2*(dx*k-dt)^2/sigma^2);
% end
pz_mx = normpdf(x,mu-param.rangelim,sigma);
pz_mx(1:nr-5) = 0;
% pz_mx = pz_mx./(sum(pz_mx)*dx);
% pz_mx = pz_mx./max(pz_mx)*0.8;
