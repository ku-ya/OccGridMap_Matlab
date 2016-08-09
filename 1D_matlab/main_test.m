% 1D case for exact inverse sensor model occGrid
% parameters
nm = 55;

dx = 1/nm;
m_cm = dx/2:dx:dx*nm;
ogmap = 0.5*ones(1,nm);
% ogmap = sensorFM(0.1, m_cm, 0.7);

% create measurements and pose
Z_1t = [0.5];
X_1t = [0.1];
%%
nz = length(Z_1t);
for j = 1:nz
    % get reduced map
    [idx idx] = min(abs(m_cm - X_1t(j)));
    [idz idz] = min(abs(m_cm - X_1t(j) - Z_1t(j)));
    
    rtl = ogmap(idx:idz);
    nr = length(rtl); % index(rtl);
    
    pr_xz = 0;pnr_xz = 1; prnp_xz = 1;
    
    pz_xr = sensorFM(nr,dx,Z_1t(j))/dx;
    for k = 1:nr
        %        pz_xrlk
        
        if k == 1
            a(k) = 0; b(k) = 1;
        else
            a(k) = a(k-1) + b(k-1)*pz_xr(k)*rtl(k);
            b(k) = b(k-1)*(1-rtl(k));
        end
    end
    c(nr) = 0;
    for j = 1:nr-1
        k = nr - j;
        c(k) = rtl(k + 1)/(1 - rtl(k))*c(k +1)+b(k)*pz_xr(k + 1)*rtl(k + 1);
    end
    for k = 1:nr
        pr_zxz(k) = a(k) + b(k)*pz_xr(k);
        pnr_zxz(k) = a(k) + b(k);
    end
    
    for k = 1:nr
        e = rtl(k)*pr_zxz(k);
        f = rtl(k)*pnr_zxz(k);
        rtl(k) = e/(e+f);
    end
    
    for k = idx:idz
        ogmap(k) = rtl(k - idx + 1);
    end
    
end

% The final grid map:
% figure,
subplot(2,1,1)
imagesc(ogmap);
colormap(flipud(gray)); axis equal;
subplot(2,1,2)
plot(m_cm,ogmap)
