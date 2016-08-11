% 1 beam case for exact inverse sensor model occGrid
% parameters
function ogmap = EISM(ogmap,range,free,X_t,param)
% L = 1; % world size
dx = param.resol; % grid size
sigma = 0.05; % sensor
% create measurements and pose
%%
% nz = length(free);
% idx = zeros(1,nz); idz = idx;
% for j = 1:nz
% get reduced map
%     [~, idx(j)] = min(abs(m_cm - X_t(j)));
%     [~, idz(j)] = min(abs(m_cm - X_t(j) - Z_t(j)));

if size(free,1)<25
    return;
end


Prtl = zeros(length(free),1);
for k = 1:length(free)
    Prtl(k) = ogmap(k);
end
nr = length(Prtl);
% Initialize
Prtl(1) = 0;
Prtl(nr+1) = 1;
% get distance to each gird in rtl
distance = sqrt((free(:,1)-X_t(1)).^2+(free(:,2)-X_t(2)).^2);
pz_xr = sensorFM(nr,param.resol,range,sigma); % forward sensor model PDF
% clf
% plot((1:nr+1)/dx,pz_xr)

a = zeros(1,nr); b = a; c = a; d = a;
for k = 1:nr
    if k == 1
        a(1) = 0; b(1) = 1;c(1)=pz_xr(1);
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
    ogmap(k) = e/(e+f);
end


for k = 1:size(free,1)
    x(k) = ogmap(k);
end


% clf
% plot(x)
% pause(0.5)
% end


