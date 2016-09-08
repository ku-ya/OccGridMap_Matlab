% 1 beam case for exact inverse sensor model occGrid
% parameters
function ogmap = AISM(ogmap,range,free,X_t,param)
% L = 1; % world size
% dx = param.resol; % grid size
% create measurements and pose
%%
% nz = length(free);
% idx = zeros(1,nz); idz = idx;
% for j = 1:nz
% get reduced map
%     [~, idx(j)] = min(abs(m_cm - X_t(j)));
%     [~, idz(j)] = min(abs(m_cm - X_t(j) - Z_t(j)));

if size(free,1)<20
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
pz_xr = sensorFM(nr,param.resol,range,param); % forward sensor model PDF

sigma = param.sigma;
rho = param.rho;
pr_zx = zeros(nr,1);
for k = 1:nr
    if k<=ceil((range-param.rangelim)/range*nr)
        pr_zx(k) = 0.3+(rho/(sigma*sqrt(2*pi()))+0.2)*exp(-1/2*(((range-param.rangelim-range*k/nr))/sigma)^2);
    else 
        pr_zx(k) = 0.5+(rho/(sigma*sqrt(2*pi())))*exp(-1/2*((range-param.rangelim-range*k/nr)/sigma)^2);
    end
end

for k = 1:length(Prtl)-1
    ogmap(k) = real(log(Prtl(k)/(1-Prtl(k))) + log(pr_zx(k)/(1-pr_zx(k))));
end

ogmap = 1 - 1./(1+exp(ogmap));


