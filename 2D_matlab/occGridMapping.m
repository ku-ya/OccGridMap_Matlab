% Robotics: Estimation and Learning
% WEEK 3
%
% Complete this function following the instruction.
function myMap = occGridMapping(ranges, scanAngles, pose, param)


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
        myMaptemp = EISM(myMaptemp,ranges(k,j),[freex,freey],xtg,param);
        for l = 1:length(freex)
            if myMap(freex(l),freey(l))<0.65
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
    myMap(myMap>0.65) = 1;
    %
    %     % Visualize the map as needed
    %
%     imagesc(myMap);colormap(flipud(gray));
%     axis equal;
%     pause(0.05)
    
    %     plot(lidar_local(:,1)+xt(1),lidar_local(:,2)+xt(2),'-x'); hold on;
    %     pause(0.2)
    %
end

end

