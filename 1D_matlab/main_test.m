% 1D case for exact inverse sensor model occGrid
% parameters
nz = 1;
nm = 10;
xt = 0;

dx = 1/nm;
m_cm = dx/2:dx:dx*nm;
ogmap = 0.5*ones(1,nm);
ogmap = sensorFM(0.1, m_cm, 0.7);

%%
% for j = 1:nz
%     nr = index(rtl);
%     for k = 1:nr
%        pz_xrlk
%        if k = 1;
%            a(k) = 0; b(k) = 1;
%        else
%           a(k) = a(k-1) + b(k-1)*pz_rxt(t)*ogmap(k);
%           b(k) = b(k-1)*(1-ogmap(k));
%        end
%     end
%     nrlp = 0;
%     for k = 1:nr
%         c(nr - k + 1) = 
%     end
%     for k = 1:nr
%         
%     end
%     
% end

% The final grid map:
% figure,
imagesc(ogmap);
colormap(flipud(gray)); axis equal;
