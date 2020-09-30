thetas = {0,0.5,1};
theta = 0.05;

lowIdx = 1;
upIdx = 1;
for ti = 1:length(thetas)-1
    if theta > thetas{ti} && ...
            theta < thetas{ti+1}
        lowIdx = ti;
        upIdx = ti+1;
        break;
    elseif theta == thetas{ti+1}
        lowIdx = ti+1;
        upIdx = ti+1;
        break;
    end
end

lowIdx
upIdx

upval = thetas{upIdx};
lowval = thetas{lowIdx};

len = abs(upval - lowval);
val = (theta - lowval) / len


alpha_up = val
alpha_low = (1-val)
