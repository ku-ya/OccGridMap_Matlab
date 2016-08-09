function pz_mx = sensorFM(nr,dx,dt)
sigma = 0.03;
for k = 1:nr
    pz_mx(k) = 1/sqrt(2*pi*sigma*2)*exp(-1/2*(dx*k-dt)^2/sigma^2);
end