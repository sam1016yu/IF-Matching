function [cand_points_edges] = mapCandidate(trajactory,road_network,...
    road_cells,road_ids_all,search_radius,cell_size,grid_size,top_k)
% find all candidate points
rows_trajactory = size(trajactory,1);
cand_points_edges = zeros(top_k*rows_trajactory,3);
for point_idx = 1:rows_trajactory
    %     fprintf('Mapping candidates %i of %i \n',point_idx,height(trajactory));
    lon = trajactory(point_idx,3);
    lat = trajactory(point_idx,4);
    [road_ids, candPoints] = getCandidatePoints(lon,lat,...
        road_network,road_cells,road_ids_all,search_radius,cell_size,grid_size,top_k);
    rows_idx = (5*point_idx-4):5*point_idx;
    cand_points_edges(rows_idx,1:2) = candPoints;
    cand_points_edges(rows_idx,3) = road_ids;
end
    trajactory = cell2mat(arrayfun(@(x) reshape(repmat(trajactory(:,x),1,top_k)',top_k*rows_trajactory,1),1:7,'UniformOutput',false));
    cand_points_edges = [cand_points_edges trajactory];
end

function [road_ids,candidate_points] = getCandidatePoints(lon,lat,...
    road_network,road_cells,road_ids_all,search_radius,cell_size, grid_size,top_k)
%find candidate points within radius r
% Parameters:
% lon,lat: longitude and latitude of the point to be searched
% raod: road network information. (outputs from splitRoad2Cell)
% grid: grid info (outputs from splitRoad2Cell)
% search_radius: the radius(km) within which candidate points are selected
% cell size: size of cutted cell in grid
% grid size: m*n array
%
% Output:
% vectors of candidate points ,in each row: [lon lat]

if nargin < 8
    top_k = 5;
end
%% search for candidate edges within given grid
% locate center grid first
center_cell_id = find(road_cells(:,4)<=lat & road_cells(:,5)>= lat & ...
    road_cells(:,2) <= lon & road_cells(:,3) >= lon);
%% search surrounding grids
extra_grids_count = ceil(search_radius/cell_size)+1;
road_ids = searchSurroundingGrids(grid_size,center_cell_id,extra_grids_count,road_cells,road_ids_all,top_k);
%% project point to candidate edges to find candidate points
candidate_edges = road_network(ismember(road_network(:,1),road_ids),:);
[rows_candidate_edges,~] = size(candidate_edges);
candidate_points = zeros(rows_candidate_edges,2);flags = zeros(rows_candidate_edges,1);
road_ids = candidate_edges(:,1);
% figure;
for edge_idx = 1 : rows_candidate_edges
    [candidate_points(edge_idx,:),flags(edge_idx)] = project2Line(lon,lat,candidate_edges(edge_idx,:));
end
% hold off;
% close all;
row_p = size(candidate_points,1);
if row_p > top_k
    res_table = [candidate_points,road_ids,1-flags,distance(fliplr(candidate_points),repmat([lat lon],row_p,1));];
    res_table = sortrows(res_table,[5,4]);
    candidate_points = res_table(1:top_k,1:2);
    road_ids = res_table(1:top_k,3);
end
% h1 = plot(lon,lat,'ro');
% h3 = text(candidate_points(:,1)',candidate_points(:,2)',{'1','2','3','4','5'});
% delete(h1);delete(h3);
end

function [candidate,flg] = project2Line(lon,lat,edge)
% see detail in:
% http://blog.sina.com.cn/s/blog_5d5c80840101bnhw.html
% http://s3.sinaimg.cn/orignal/5d5c8084gd638da294362&690
a = [edge(:,4) edge(:,5)];
b = [edge(:,6) edge(:,7)];
p = [lon lat];
ap = p-a; ab = b-a; r = sum(ap.*ab)/sum(ab.^2);
if r < 0
    candidate = a;
    flg = 0;
elseif r> 1
    candidate = b;
    flg = 0;
else
    candidate = r*ab + a;
    flg = 1;
end
% hold on;
% plot(lon,lat,'*');
% plot([a(1),b(1)],[a(2),b(2)],'o-');plot(candidate(1),candidate(2),'x');
% axis equal;
% hold off;cla;
end

function [road_ids] = searchSurroundingGrids(grid_size,center_cell_id,search_steps,grid,road_ids_all,top_k)
%searchSurroundingGrids
%   search surrounding edges for candidate edges
%   Input:
%   grid_size: m*n array
%   search_steps: steps that search is conducted
%   grid: grided road network info

% consider looking around center grid based on the given search radius and
% grid size used previously
grid_cols = grid_size(2);grid_rows = grid_size(1);

% function that convert between cellId and  [row_id,col_id]
% convenient for searching surrounding grids
cell2rowcol = @(cellid) [ceil(cellid/grid_cols), ...
    mod(cellid,grid_cols)+(mod(cellid,grid_cols)==0)*grid_cols];
rowcol2cell = @(row,col) grid_cols*(row-1) + col;
isValidRowCol = @(row,col) row>0 && row<=grid_rows && col>0 && col <= grid_cols;

road_ids = road_ids_all{center_cell_id,1};
center_rowcol = cell2rowcol(center_cell_id);
center_row = center_rowcol(1); center_col = center_rowcol(2);
if search_steps > 0
    for step_col = -search_steps:search_steps
        for step_row = -search_steps:search_steps
            if isValidRowCol(center_row+step_row,center_col+step_col)
                road_ids = [road_ids road_ids_all{rowcol2cell(center_row+step_row,center_col+step_col),1}];
            end
        end
    end
end
road_ids = unique(road_ids)';
if length(road_ids) < top_k
    road_ids = road_ids';
    if search_steps < 0
        search_steps = 1;
    else
        search_steps = search_steps + 1;
    end
    while length(road_ids) < top_k
        for step_col = -search_steps:search_steps
            for step_row = -search_steps:search_steps
                if isValidRowCol(center_row+step_row,center_col+step_col)
                    roads_in_cell = road_ids_all{rowcol2cell(center_row+step_row,center_col+step_col),1};
                    if ~isempty(roads_in_cell)
                        road_ids = reshape(road_ids,1,[]);
                        road_ids = [road_ids roads_in_cell];
                    end
                end
            end
        end
        road_ids = unique(road_ids)';
        search_steps = search_steps + 1;
    end
end
end


