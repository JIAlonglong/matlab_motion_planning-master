function hybridAStar()
    % Define the grid size and resolution
    gridSize = [20, 20];
    resolution = 1;
    
    % Define the start and goal positions and orientations
    start = [1, 1, 0];  % [x, y, theta]
    goal = [15, 15, pi/2];  % [x, y, theta]
    
    % Define the obstacles
    obstacles = [5, 5; 5, 6; 6, 5; 7, 8; 10, 10; 11, 11];
    
    % Create the grid
    grid = ones(gridSize);
    for i = 1:size(obstacles, 1)
        grid(obstacles(i, 1), obstacles(i, 2)) = 0;
    end
    
    % Initialize open and closed lists
    openList = [];
    closedList = zeros(gridSize);
    
    % Initialize the start node
    startNode = createNode(start, [], 0, heuristic(start, goal));
    openList = [openList; startNode];
    
    % Initialize the figure for animation
    figure;
    axis equal;
    axis([1 gridSize(1) 1 gridSize(2)]);
    set(gca, 'YDir', 'reverse');
    hold on;
    colormap(gray);
    imagesc(grid');
    plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goal(1), goal(2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Variable to store frames for GIF
    frames = [];
    filename = 'hybrid_astar_animation.gif';
    
    while ~isempty(openList)
        % Find the node with the lowest f cost
        [~, idx] = min([openList.f]);
        currentNode = openList(idx);
        openList(idx) = [];
        
        % Check if we have reached the goal
        if distance(currentNode.position, goal) < resolution
            path = reconstructPath(currentNode);
            plotPath(grid, path, start, goal, frames, filename);
            return;
        end
        
        % Add the current node to the closed list
        closedList(round(currentNode.position(1)), round(currentNode.position(2))) = 1;
        
        % Get the neighbors of the current node
        neighbors = getNeighbors(currentNode.position, gridSize, resolution);
        for i = 1:size(neighbors, 1)
            neighborPos = neighbors(i, :);
            
            if isObstacle(neighborPos, grid)
                continue; % Skip obstacles
            end
            
            gCost = currentNode.g + distance(currentNode.position, neighborPos);
            hCost = heuristic(neighborPos, goal);
            neighborNode = createNode(neighborPos, currentNode, gCost, hCost);
            
            if isNodeInList(neighborNode, openList)
                existingNode = openList(isNodeInList(neighborNode, openList));
                if neighborNode.g < existingNode.g
                    openList(isNodeInList(neighborNode, openList)) = neighborNode;
                end
            else
                openList = [openList; neighborNode];
            end
        end
        
        % Plot the current node
        plot(currentNode.position(1), currentNode.position(2), 'rx');
        
        % Capture the frame for the GIF
        frame = getframe(gcf);
        frames = [frames, frame];
    end
    
    disp('No path found');
end

function node = createNode(position, parent, g, h)
    node.position = position;
    node.parent = parent;
    node.g = g;
    node.h = h;
    node.f = g + h;
end

function h = heuristic(pos, goal)
    h = norm(pos(1:2) - goal(1:2)); % Euclidean distance
end

function neighbors = getNeighbors(position, gridSize, resolution)
    directions = [resolution, 0, 0; -resolution, 0, 0; 0, resolution, pi/4; 0, -resolution, -pi/4];
    neighbors = [];
    
    for i = 1:size(directions, 1)
        newPos = position + directions(i, :);
        if newPos(1) >= 1 && newPos(1) <= gridSize(1) && ...
           newPos(2) >= 1 && newPos(2) <= gridSize(2)
            neighbors = [neighbors; newPos];
        end
    end
end

function isObs = isObstacle(position, grid)
    x = round(position(1));
    y = round(position(2));
    if x < 1 || x > size(grid, 1) || y < 1 || y > size(grid, 2)
        isObs = true;
    else
        isObs = grid(x, y) == 0;
    end
end

function path = reconstructPath(node)
    path = [];
    while ~isempty(node)
        path = [node.position; path];
        node = node.parent;
    end
end

function isInList = isNodeInList(node, list)
    isInList = false;
    for i = 1:length(list)
        if isequal(list(i).position, node.position)
            isInList = i;
            break;
        end
    end
end

function plotPath(grid, path, start, goal, frames, filename)
    plot(path(:,1), path(:,2), 'r', 'LineWidth', 2);
    
    % Capture the final frame with the path
    frame = getframe(gcf);
    frames = [frames, frame];
    
    % Save the frames as a GIF
    for i = 1:length(frames)
        [imind, cm] = rgb2ind(frames(i).cdata, 256);
        if i == 1
            imwrite(imind, cm, filename, 'gif', 'Loopcount', inf);
        else
            imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append');
        end
    end
end

function dist = distance(pos1, pos2)
    dist = norm(pos1(1:2) - pos2(1:2));
end

% Run the Hybrid A* algorithm
hybridAStar();
