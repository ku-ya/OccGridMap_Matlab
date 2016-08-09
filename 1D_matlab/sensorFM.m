function pz_mx = sensorFM(x,m_cm,dt)
sigma = 0.1;
for k = 1:length(m_cm)
    pz_mx(k) = 1/sqrt(2*pi*sigma*2)*exp(-1/2*(m_cm(k)-x-dt)^2/sigma^2);
end