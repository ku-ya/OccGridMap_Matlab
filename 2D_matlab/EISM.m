% 1 beam case for exact inverse sensor model occGrid
% parameters
function ogmap = EISM(ogmap,free,X_t,param)

L = 1; % world size
dx = L/nm; % grid size
sigma = L*0.01; % sensor

% create measurements and pose

%%
nz = length(Z_t);
idx = zeros(1,nz); idz = idx;
for j = 1:nz
    % get reduced map
    [~, idx(j)] = min(abs(m_cm - X_t(j)));
    [~, idz(j)] = min(abs(m_cm - X_t(j) - Z_t(j)));
    Prtl = ogmap(idx(j):idz(j));
    nr = length(Prtl);
    
    % Initialize
    Prtl(1) = 0;
    Prtl(nr+1) = 1;
    pz_xr = sensorFM(nr + 1,dx,Z_t(j),sigma); % forward sensor model PDF
    
    for k = 1:nr
        if k == 1
            a(k) = 0; b(k) = 1;c(1)=pz_xr(1);
        else
            a(k) = a(k-1) + b(k-1)*pz_xr(k-1)*Prtl(k-1);
            b(k) = b(k-1)*(1-Prtl(k-1));
            c(k) = b(k)*pz_xr(k);
        end
    end
    d(nr) = 0;
    for p = 1:nr-1
        k = nr - p;
        d(k) = d(k+1) + b(k)*pz_xr(k + 1)*Prtl(k + 1);
    end
    for k = 1:nr
        Pr_zxz(k) = a(k) + c(k);
        Pnr_zxz(k) = a(k) + d(k);
    end
    
    for k = 1:nr
        e = Prtl(k)*Pr_zxz(k);
        f = (1-Prtl(k))*Pnr_zxz(k);
        Prtl(k) = e/(e+f);
    end
    ogmap(idx(j):idx(j)+length(Prtl)-2) = Prtl(1:end-1);
    
end


