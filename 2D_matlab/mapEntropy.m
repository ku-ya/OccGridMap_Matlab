function H = mapEntropy(map)
[indx, indy] = size(map);
H = 0;
for k = 1:indx
    for j = 1:indy
        p = map(k,j);
 
            if p >= 1-eps || p <= eps
                H = H;
            else
                H = H - (p*log(p) + (1-p)*log(1-p));
            end

    end
end
H = H/(indx*indy);
