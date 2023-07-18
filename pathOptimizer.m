% TO-DO
% 1) Add in Level requirements
% 2) Add in Fast travel consideration
% 3) Update cost function for both distance and level discrepancy
% 4) Add in Prerequisite node requirements
% 5) Refactor solver() to while loop for solution convergence

try
    [nodeTable, startNode, pathType, mapImg] = problemSetup();
catch
    disp('Program canceled.')
    return
end

path = solver(nodeTable,startNode,pathType);

outFile = createPathPlot(nodeTable,startNode,path,pathType,mapImg);
writePathOutput(path,pathType,outFile);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [nodeTable, startNode, pathType, mapImg] = problemSetup()
    clc    

    disp(['Select node file to import and optimize...' newline])

    [inputFile,inputPath] = uigetfile({'*.csv','CSV (Comma-delimited) (*.csv)';'*.*','All files (*.*)'},'Select Node File to Import','C:\');

    if isequal(inputFile,0)
        clc
        disp('Program canceled.');
        return
    else
        fullInputPath = fullfile(inputPath,inputFile);
    end

    try
        opts = detectImportOptions(fullInputPath);
        
        opts.VariableNames = {'Name','X','Y','isRepeatable','Level','Requires','FastTravelsTo'};
        opts.VariableTypes = {'string','double','double','double','double','string','string'};
        
        nodeTable = readtable(fullInputPath,opts);
        
        nodeTable.Node = zeros(length(nodeTable.Name),1);
        nodeTable.isStart = zeros(length(nodeTable.Node),1);
    catch
        clc
        disp(['Input file is not formatted properly. File must have the following data:' newline newline '[Node ID, Node Name, X-Coord, Y-Coord, is Repeatable (bool), is Start Node (bool)]'])
    end

    lenNodes = length(nodeTable.Name);
    
    for ii = 1:lenNodes
        nodeTable.Node(ii) = ii;
    end

    nodeTable = movevars(nodeTable,"Node",'Before',1);

    clc
    startNodeInput = input("Enter the Name for the starting node: ","s");

    startNode = find(nodeTable.Name==startNodeInput);

    if (length(startNode)>1)
        clc
        disp(['More than one node has the entered name. Make sure that every node has a unique name and that the name entered is spelled correctly.' newline newline])
        return
    elseif (isempty(startNode))
        clc
        disp(['A node with a name matching the entered name does not exist. Make sure that every node has a unique name and that the name entered is spelled correctly.' newline newline])
        return
    end
    
    nodeTable.isStart(startNode) = 1;
    
    mapImg = importMap();

    inputErr = 1;

    while(inputErr)
        pathType = input("Output a closed-loop or open-loop solution? [closed/open]: ","s");

        if (pathType == "closed" || pathType == "c")
            inputErr = 0;
            pathType = "closed";
            disp([newline 'Generating solution...' newline])
        elseif (pathType == "open" || pathType == "o")
            inputErr = 0;
            pathType = "open";
            disp([newline 'Generating solution...' newline])
        else
            inputErr = 1;
            disp(['Unrecognized selection.' newline])
        end
    end
end

function mapOut = importMap()
    disp(['Select node map image file to import...' newline])

    [inputFile,inputPath] = uigetfile({'*.png','PNG (*.png)';'*.*','All files (*.*)'},'Select Map File to Import','C:\');

    if isequal(inputFile,0)
        clc
        disp('Program canceled.');
        return
    else
        mapFile = fullfile(inputPath,inputFile);
    end
    
    mapOut = imread(mapFile,'png');
end

function [newPath, minCost] = insertNode(currentPath,insertedNode,type)
    if (type=="closed")
        testPath = [currentPath(1,:);insertedNode;currentPath(2:end,:)];
        minCost = pathCost(testPath);
        newPath = testPath;

        for ii = 2:(length(currentPath(:,1))-1)
            if (currentPath{(ii+1),1}~=insertedNode{1,1} && currentPath{ii,1}~=insertedNode{1,1})
                testPath = [currentPath(1:ii,:);insertedNode;currentPath((ii+1):end,:)];
                newCost = pathCost(testPath);

                if (newCost < minCost)
                    newPath = testPath;
                    minCost = newCost;
                end
            else
                continue
            end
        end
    else
        if (length(currentPath(:,1))==1)
            testPath = [currentPath(1,:);insertedNode];
        else
            testPath = [currentPath(1,:);insertedNode;currentPath(2:end,:)];
        end
        
        minCost = pathCost(testPath);
        newPath = testPath;

        for ii = 2:(length(currentPath(:,1)))
            if (ii==length(currentPath(:,1)))
                testPath = [currentPath;insertedNode];
            else
                testPath = [currentPath(1:ii,:);insertedNode;currentPath((ii+1):end,:)];
            end
            
            newCost = pathCost(testPath);

            if (newCost < minCost)
                newPath = testPath;
                minCost = newCost;
            end
        end
    end
end

function cost = pathCost(path)
    cost = 0;
    for ii = 1:(length(path(:,1))-1)
        cost = cost + nodeDistance(path(ii,:),path((ii+1),:));
    end
end

function d = nodeDistance(node1, node2)
    if (node1{1,1} == node2{1,1})
        d = 0;
    else
        node1X = node1{1,3}(1);
        node1Y = node1{1,3}(2);
        node2X = node2{1,3}(1);
        node2Y = node2{1,3}(2);
        
        d = sqrt((node2X-node1X)^2 + (node2Y-node1Y)^2);
    end
end

function filepath = createPathPlot(nodeTable, startNode, path, type, map) 
    solnFig = figure('Name','Path Solution','NumberTitle','off','Visible','off');
    image(map);
    
    hold on

    ax = gca;

    set(ax,'YDir','normal');
    
    ax.XTick = [];
    ax.YTick = [];
    ax.XTickLabels = {};
    ax.YTickLabels = {};
    
    axPosition = ax.Position;
    
    for ii = 1:(length(path(:,1))-1)
        X = (([path{ii,3}(1) path{(ii+1),3}(1)]) * (axPosition(3) / length(map(1,:,1)))) + axPosition(1);
        Y = (([path{ii,3}(2) path{(ii+1),3}(2)]) * (axPosition(4) / length(map(:,1,1)))) + axPosition(2);
        
        annotation(solnFig, 'arrow',X,Y,'Units','pixels','Color','m','LineWidth',2);
    end
    
    scatter(nodeTable.X(:),nodeTable.Y(:),'o','filled','b');

    scatter(nodeTable.X(startNode),nodeTable.Y(startNode),'o','filled','g');
    
    if (type=="open")
        scatter(path{end,3}(1), path{end,3}(2),'o','filled','r');
    end
    
    figPath = uigetdir('C:\','Save Solution Plot');

    if isequal(figPath,0)
        clc
        disp('Solution plot save canceled.');
        return
    else
        fullFigPath = fullfile(figPath,strcat("pathOptimizerSolution-",string(datetime('now'),"yyyy-MM-dd-HH-mm-ss")));
        saveas(solnFig,fullFigPath,'png');
        fullFigPath = strcat(fullFigPath,".png");
        
        filepath = fullFigPath;
        
        if ismac
            % Code to run on Mac platform
        elseif isunix
            % Code to run on Linux platform
        elseif ispc
            winopen(fullFigPath);
        end
    end
end

function pathOut = solver(nodeTable, startNode, type)
    lenCoords = length(nodeTable.Node(:));
%     loopLim = lenCoords*3;
    loopLim = 5;
    
    for ii = 1:loopLim
        clc
        disp(['Start Node: ' char(nodeTable.Name(startNode)) newline 'Solution Type: ' char(type) newline newline 'Generating solution... '])
        disp([num2str(int64((ii/loopLim)*100)) '%'])
        
        unvisitedNodes = cell.empty;

        unvisitedItr = 1;
        repeatableCounter = 0;
        for tableIdx = 1:lenCoords
            if (tableIdx ~= startNode)
                unvisitedNodes{unvisitedItr,1} = nodeTable.Node(tableIdx);
                unvisitedNodes{unvisitedItr,2} = nodeTable.Name(tableIdx);
                unvisitedNodes{unvisitedItr,3} = [nodeTable.X(tableIdx) nodeTable.Y(tableIdx)];
                unvisitedNodes{unvisitedItr,4} = nodeTable.isRepeatable(tableIdx);
                
                if (nodeTable.isRepeatable(tableIdx) == 1)
                    repeatableCounter = repeatableCounter + 1;
                    repeatableVisited{repeatableCounter,1} = nodeTable.Node(tableIdx);
                    repeatableVisited{repeatableCounter,2} = 0;
                end

                unvisitedItr = unvisitedItr + 1;
            end
        end

        numRepeatable = sum(nodeTable.isRepeatable(:));

        unvisitedLen = length([unvisitedNodes{:,1}]);
        
        if (type == "closed")
            path = {nodeTable.Node(startNode) nodeTable.Name(startNode) [nodeTable.X(startNode) nodeTable.Y(startNode)];nodeTable.Node(startNode) nodeTable.Name(startNode) [nodeTable.X(startNode) nodeTable.Y(startNode)]};
        else
            path = {nodeTable.Node(startNode) nodeTable.Name(startNode) [nodeTable.X(startNode) nodeTable.Y(startNode)]};
        end
        
        allRepeatsVisited = 0;
        while (unvisitedLen > numRepeatable || ~allRepeatsVisited)
            randIdx = randi(unvisitedLen);
            randNode = unvisitedNodes(randIdx,1:3);
            isRepeatable = unvisitedNodes{randIdx,4};
            
            [path, newPathCost] = insertNode(path,randNode,type);
            
            if (~isRepeatable)
                newUnvisitedIdx = [1:(randIdx-1) (randIdx+1):unvisitedLen];
                unvisitedNodes = unvisitedNodes(newUnvisitedIdx,:);
                unvisitedLen = length([unvisitedNodes{:,1}]);
            else
                repeatVisitedIdx = find([repeatableVisited{:,1}]==randNode{1,1});
                repeatableVisited{repeatVisitedIdx,2} = repeatableVisited{repeatVisitedIdx,2} + 1;
                
                if (length(find([repeatableVisited{:,2}]>0)) >= numRepeatable)
                    allRepeatsVisited = 1;
                end
            end
        end

        if((ii==1) || (ii>1 && newPathCost<finalPathCost))
            finalPathCost = newPathCost;
            finalPath = path;
        end
    end
    
    pathOut = finalPath;
end

function writePathOutput(path,type,filepath)
    clc

    if (type == "open")
        disp(['OPEN-LOOP SOLUTION: ' newline])
    else
        disp(['CLOSED-LOOP SOLUTION: ' newline])
    end
    
    for ii = 1:10
        disp(['Stop ' num2str(ii) ': ' char(path{ii,2}) ' [' num2str(path{ii,3}(1)) ', ' num2str(path{ii,3}(2)) ']'])
    end
    
    disp(['...' newline newline 'Full path solution saved to:']);
    disp(filepath)
    disp(newline)
end