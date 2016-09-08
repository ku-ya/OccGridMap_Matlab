% Robotics: Estimation and Learning
% WEEK 3
%
% Complete this function following the instruction.
function [myMap, H, IG]= occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters
%
% % the number of grids for 1 meter.
myResol = param.resol;
% % the initial map size in pixels
myMap = 0.5*ones(param.size);
% % the origin of the map in pixels
myorigin = param.origin;
%
% % 4. Log-odd parameters
% lo_occ = param.lo_occ;
% lo_free = param.lo_free;
% lo_max = param.lo_max;
% lo_min = param.lo_min;


N = size(pose,2);
for j = 1:N % for each time,
    xt = pose(:,j);
    
    lidar_local_plt = [ranges(:,j).*cos(scanAngles + xt(3)) -ranges(:,j).*...
        sin(scanAngles + xt(3))];
    
    ranges(:,j) = ranges(:,j) + param.rangelim;
    lidar_local = [ranges(:,j).*cos(scanAngles + xt(3)) -ranges(:,j).*...
        sin(scanAngles + xt(3))];
    
    xtg = [ceil(xt(1)*myResol)+myorigin(1),ceil(xt(2)*myResol)+...
        myorigin(2)];
    
    myMap(xtg(1),xtg(2)) = 0;
    %
    %
    %     % Find grids hit by the rays (in the gird map coordinate)
    for k = 1:length(scanAngles)-1
        rtl = ceil(lidar_local(k,:)*param.resol);
        
        [freex, freey] = bresenham(xtg(1),xtg(2),xtg(1)+rtl(1),...
            xtg(2)+rtl(2));
        %
        %         % Find occupied-measurement cells and free-measurement cells
        %         % convert to 1d index
        %          free = sub2ind(size(myMap),freey,freex);
        %         % set end point value
        %         map(occ(2),occ(1)) = 3;
        %         % set free cell values
        %          myMap(free) = 1;
        myMaptemp = ones(length(freex),1);
        for l = 1:length(freex)
            myMaptemp(l) = myMap(freex(l),freey(l));
        end
        
        if param.ISM == 'EISM'
            myMaptemp = EISM(myMaptemp,ranges(k,j),[freex,freey],xtg,param);
        elseif param.ISM == 'AISM'
            myMaptemp = AISM(myMaptemp,ranges(k,j),[freex,freey],xtg,param);
        end
        for l = 1:length(freex)
            if myMap(freex(l),freey(l))< param.lo_occ
                myMap(freex(l),freey(l))=myMaptemp(l);
            else
                break
            end
        end
    end
    %     % Update the map
    %
    %
    %     % Saturate the map?
    %
    myMap(myMap>=param.lo_occ) = 1;
    %
    %     % Visualize the map as needed
    if param.fig == 1
        caxis([0.2 0.8])
        imagesc(1-myMap);colormap('gray');hold on
        % Make a truecolor all-green image.
        green = cat(3, zeros(size(myMap)),...
            ones(size(myMap)), zeros(size(myMap)));
        hold on;
        h = imshow(green);
        hold off;
        % Use our influence map as the
        % AlphaData for the solid green image.
        I = zeros(size(myMap));
        for k = 1:length(lidar_local_plt)
            I(ceil((lidar_local_plt(k,1)+xt(1))*param.resol)+param.origin(1),...
                ceil((lidar_local_plt(k,2)+xt(2))*param.resol)+param.origin(2))=1;
        end
        set(h, 'AlphaData', I)
        axis equal;hold on;
        plot(xt(2)*param.resol+param.origin(2),xt(1)*param.resol+param.origin(1),'ro','linewidth',2,'MarkerSize',8);
        axis tight;
        
        pause(0.05)
        
    end
    
    H(j) = mapEntropy(myMap);
    
    if j==1
        IG(1) = 0;
    else
        IG(j) = H(j-1) - H(j);
    end
%     if param.ISM == 'EISM'
%         x = 1
%     end
    %     plot(lidar_local(:,1)+xt(1),lidar_local(:,2)+xt(2),'-x'); hold on;
    %     pause(0.2)
    %
end

end

