% 1D case for exact inverse sensor model occGrid
% parameters
clear all;
clf;clc;
nm = 101; % number of grid nm
L = 1; % world size
dx = L/nm; % grid size
m_cm = dx/2:dx:dx*nm; % grid center of mass
ogmap = 0.2*ones(1,nm); % map
sigma = L*0.05; % sensor

% create measurements and pose
Z_1t = ones(10,1)*L*0.5;
X_1t = ones(10,1)*L*0.1;

%%
nz = length(Z_1t);
idx = zeros(1,nz); idz = idx;
for j = 1:nz
    % get reduced map
    [~, idx(j)] = min(abs(m_cm - X_1t(j)));
    [~, idz(j)] = min(abs(m_cm - X_1t(j) - Z_1t(j)));
%     idz(j) = idz(j) + round(sigma*3/dx);
    Prtl = ogmap(idx(j):idz(j));
    nr = length(Prtl);
    
    % Initialize
    Prtl(1) = 0;
    Prtl(nr+1) = 1;
    pz_xr = sensorFM(nr + 1,dx,Z_1t(j),sigma); % forward sensor model PDF
    
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
    
    % The final grid map:
    % figure,
    subplot(2,1,1)
    imagesc(ogmap);
    colormap(flipud(gray)); axis equal;
    ylim([-5 5]);
    subplot(2,1,2)
    plot(m_cm(idx(j):idz(j)),zeros(idz(j)-idx(j)+1),'r','linewidth',2); hold on;
    plot(m_cm,ogmap,'b-');
    plot(X_1t(j),0,'>m','MarkerSize',15,'linewidth',2)
    ylim([0 1]);
     pause(0.1);
end


