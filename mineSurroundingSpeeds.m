function surroundingSpeeds = mineSurroundingSpeeds(cand_points_edges)
%mine surrounding speeds of each candidate road segments
rows_trajactory = size(cand_points_edges,1); 

search_range = 2*60*60 / 6;

surroundingSpeeds = zeros(rows_trajactory,1);
for cand_idx = 1:rows_trajactory
    search_start = max(1,cand_idx - round(search_range/2));
    search_end = min(rows_trajactory,cand_idx + round(search_range/2));

    search_trajactory = cand_points_edges(search_start:search_end,:);
    center_trajactory = cand_points_edges(cand_idx,:);
    
    same_road = search_trajactory(:,3) == center_trajactory(:,3);
    similar_direction = abs(search_trajactory(:,3)-center_trajactory(:,3)) < 90;
    similar_objects_speeds = search_trajactory(same_road & similar_direction,8);
    surroundingSpeeds(cand_idx) = mean(similar_objects_speeds);
end

end

