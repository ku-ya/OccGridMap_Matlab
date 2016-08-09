
zl = 2;
n = 20;
z = linspace(0,4,n);
k = 0.6;
d = 0.6;
for k = 1:size(z,2)
    if z(k)<=zl
        pm_zx(k) = 0.3 + (k/(d*sqrt(2*pi))+0.2)*exp(-1/2*((z(k)-zl)/d)^2);
    else
        pm_zx(k) = 0.5 + k/(d*sqrt(2*pi))*exp(-1/2*((z(k)-zl)/d)^2);
    end
end
plot(z,pm_zx)