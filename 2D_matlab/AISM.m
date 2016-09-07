% 1 beam case for exact inverse sensor model occGrid
% parameters
function ogmap = AISM(ogmap,range,free,X_t,param)
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
pz_xr = sensorFM(nr,param.resol,range,sigma); % forward sensor model PDF

for k = 1:length(Prtl)
    ogmap = Prtl(k) + 
end



