function [historySpeeds] = mineHistorySpeed(trajactory,cand_points_edges)
%history speed mining
historySpeeds = containers.Map('KeyType','double','ValueType','any');
rows_trajactory = size(trajactory,1);
top_k = size(cand_points_edges,1) / rows_trajactory;
% [top_k,~,rows_trajactory] = size(cand_points_edges);
cand_road_ids = cand_points_edges(:,3);
hourOfDay = 1+floor(datetime(trajactory(:,2),'ConvertFrom','posixtime').Hour/2);
hourOfDay = reshape(repmat(hourOfDay,1,top_k)',top_k*rows_trajactory,1);
speeds = reshape(repmat(trajactory(:,5),1,top_k)',top_k*rows_trajactory,1);
G = findgroups(hourOfDay,cand_road_ids);
mean_speeds_road_time = splitapply(@mean,speeds,G);
for group_idx = 1:max(G)
    loc = find(G==group_idx,1);
    road_id = cand_road_ids(loc,1);
    if ~isKey(historySpeeds,road_id)
        speedOfRoad = zeros(1,12);
    else
        speedOfRoad = historySpeeds(road_id);
    end
    speedOfRoad(hourOfDay(loc)) = mean_speeds_road_time(group_idx);
    historySpeeds(road_id) = speedOfRoad;
end
end

