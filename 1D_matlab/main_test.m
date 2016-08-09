% 1D case for exact inverse sensor model occGrid
% parameters
nz = 1;
nm = 15;
xt = 0;

dx = 1/nm;
m_cm = dx/2:dx:dx*nm;
ogmap = 0.5*ones(1,nm);
% ogmap = sensorFM(0.1, m_cm, 0.7);

%%
for m = 1:nz
    nr = 15; % index(rtl);
    pr_xz = 0;pnr_xz = 1; prnp_xz = 1;
    
    pz_xr = sensorFM(0,m_cm,0.7);
    for k = 1:nr
        %        pz_xrlk
        
        if k == 1
            a(k) = 0; b(k) = 1;
        else
            a(k) = a(k-1) + b(k-1)*pz_xr(k)*ogmap(k);
            b(k) = b(k-1)*(1-ogmap(k));
        end
    end
    c(nr) = 0;
    for j = 1:nr-1
        k = nr - j;
        c(k) = ogmap(k + 1)/(1 - ogmap(k))*c(k +1)+b(k)*pz_xr(k + 1)*ogmap(k + 1);
    end
    for k = 1:nr
        pr_zxz(k) = a(k) + b(k)*pz_xr(k);
        pnr_zxz(k) = a(k) + b(k);
    end
    
    for k = 1:nr
        e = ogmap(k)*pr_zxz(k);
        f = ogmap(k)*pnr_zxz(k);
        ogmap(k) = e/(e+f);
    end
    
end

% The final grid map:
% figure,
subplot(2,1,1)
imagesc(ogmap);
colormap(flipud(gray)); axis equal;
subplot(2,1,2)
plot(m_cm,ogmap)
