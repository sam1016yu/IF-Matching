function [F,path_edges] = fcnSpatialTemporal(ptn0_cand,ptn1_cand,ptn0_raw,...
    ptn1_raw,edg0_cand,edg1_cand,G,node_table,road_network,historySpeeds,surroundingSpeed)
%fcnSpatial:spatial analysis function
%   Implementation of equation 3 in the paper
%   Input:
%   ptn0_cand,ptn1_cand: two neighboring candidate points [lon lat]
%   ptn0_raw,ptn1_raw: GPS points of which candidate points belong
%   road_network: output of splitRoad2Cell
    
    % search for shortest path first
    [path_edges,dist_min] = findShortestPath(G,node_table,ptn0_cand,...
        edg0_cand,ptn1_cand,edg1_cand,road_network);
    if dist_min == 0
        F = 0;
    elseif dist_min == Inf
        % means no path
        F = -99999;
    else
        Fs = fcnSpatial(ptn1_cand,ptn0_raw,ptn1_raw,road_network(road_network(:,1) == edg1_cand,:),dist_min);
        Ft = fcnTemporal(ptn0_raw,ptn1_raw,historySpeeds(edg1_cand),surroundingSpeed);
        F = Fs * Ft;
    end
end

function Fs = fcnSpatial(ptn1_cand,ptn0_raw,ptn1_raw,edg1_cand,dist_shortest)
%fcnSpatial:spatial analysis function
%   Implementation of equation 3 in the paper
%   Input:
%   ptn0_cand,ptn1_cand: two neighboring candidate points [lon lat]
%   ptn0_raw,ptn1_raw: GPS points of which candidate points belong
%   road_network: output of splitRoad2Cell
%%
Ncs = fcnNormal(ptn1_cand,ptn1_raw);
% distance between two GPS points
dist_ratio = deg2km(distance(ptn0_raw(1,[2,1]),ptn1_raw(1,[2,1])))/dist_shortest;
if dist_ratio >= 1.5
    dist_ratio = 0;
%     dist_ratio = 1 + log10(dist_ratio)/4;
end
edg1_az1 = azimuth(edg1_cand(5),edg1_cand(4),edg1_cand(7),edg1_cand(6));
edg1_az2 = azimuth(edg1_cand(7),edg1_cand(6),edg1_cand(5),edg1_cand(4));
angle_difference = min(abs(edg1_az1-ptn0_raw(4)),abs(edg1_az2-ptn0_raw(4)));
selection_prob = 1 - angle_difference/360;

Fs = Ncs*dist_ratio*selection_prob;
end


function Ft = fcnTemporal(ptn0_raw,ptn1_raw,history_speed,surrounding_speed)
    avg_speed = (ptn0_raw(3)+ptn1_raw(3))/2;
    hourOfptn1 = 1+floor(datetime(ptn1_raw(5),'ConvertFrom','posixtime').Hour/2);
    hours_ratio = 1-abs((1:12)-hourOfptn1)/24;
    v_history = sum(history_speed.*hours_ratio)/12;
    if surrounding_speed > 0
        alpha = 0.6;
    else
        alpha = 1;
    end
    beta = 1-alpha;
    % cosine distance of two speed vecotrs
    Ft = avg_speed/(alpha*v_history+beta*surrounding_speed);
end