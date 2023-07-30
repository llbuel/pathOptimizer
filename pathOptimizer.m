% TO-DO
% 2) Add in Fast travel consideration
% 3) Update cost function for both distance and level discrepancy
% 4) Add in Prerequisite node requirements
% 5) Refactor solver() to while loop for solution convergence


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MAIN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


try
    [nodeTable, startNode, pathType, mapImg] = problemSetup();
catch
    return
end

path = solver(nodeTable,startNode,pathType);

if ispc
    dir = uigetdir('C:\','Save Solution Plot');
    outPath = createPathPlot(dir,nodeTable,startNode,path,pathType,mapImg);
else
    dir = uigetdir(pwd,'Save Solution File');
    mapPath = [];
    outPath = createSolnFile(dir,path,mapPath);
end

writePathOutput(path,pathType);

disp(['...' newline newline 'Full path solution saved to:']);
disp(outPath);
disp(newline);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [nodeTable, startNode, pathType, mapImg] = problemSetup()
    clc    

    disp(['Select node file to import and optimize...' newline])
    
    if ispc
        [inputFile,inputPath] = uigetfile({'*.csv','CSV (Comma-delimited) (*.csv)';'*.*','All files (*.*)'},'Select Node File to Import','C:\');
    else
        [inputFile,inputPath] = uigetfile({'*.csv','CSV (Comma-delimited) (*.csv)';'*.*','All files (*.*)'},'Select Node File to Import',pwd);
    end

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
        
        newRequiresVec = cell(length(nodeTable.Name),1);
        
        for nodeRow = 1:length(nodeTable.Name)
            requirementStr = nodeTable.Requires(nodeRow);
            
            if (~ismissing(requirementStr))
                newRequiresVec{nodeRow,1} = [split(requirementStr,";")]';
            end
        end
        
        nodeTable.Requires = newRequiresVec;
        
        newFastTravelVec = cell(length(nodeTable.Name),1);
        
        for nodeRow = 1:length(nodeTable.Name)
            fasttravelStr = nodeTable.FastTravelsTo(nodeRow);
            
            if (~ismissing(fasttravelStr))
                newFastTravelVec{nodeRow,1} = [split(fasttravelStr,";")]';
            end
        end
        
        nodeTable.FastTravelsTo = newFastTravelVec;
        
        nodeTable.Node = zeros(length(nodeTable.Name),1);
        nodeTable.isStart = zeros(length(nodeTable.Node),1);
    catch
        clc
        disp(['Program canceled: Input file is not formatted properly.' newline 'File must have the following data columns:' newline newline '[Node Name, X-Coord, Y-Coord, Is-Repeatable (bool), Level, Required Nodes (if any), Fast Travel Nodes (if any)]'])
    end

    lenNodes = length(nodeTable.Name);
    
    for nodeItr = 1:lenNodes
        nodeTable.Node(nodeItr) = nodeItr;
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
    
    if ispc
        mapImg = importMap();
    else
        mapImg = [];
    end

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

function [newPath, minCost, isValidInsert] = insertNode(currentPath,insertedNode,type)
    if (type=="closed")
        numValid = 0;
        testPath = [currentPath(1,:);insertedNode;currentPath(2:end,:)];
        minCost = pathCost(testPath);
        newPath = testPath;
        
        if (isempty(insertedNode{1,4}{1,1}) || sum(ismember([currentPath{1,2}],insertedNode{1,4}{1,1}))==length(insertedNode{1,4}{1,1}))
            numValid = numValid + 1;
        end

        for ii = 2:(length(currentPath(:,1))-1)
            testPath = [currentPath(1:ii,:);insertedNode;currentPath((ii+1):end,:)];
            
            if (isempty(insertedNode{1,4}{1,1}) || sum(ismember([currentPath{1:ii,2}],insertedNode{1,4}{1,1}))==length(insertedNode{1,4}{1,1}))
                numValid = numValid + 1;
            else
                continue
            end
            
            newCost = pathCost(testPath);

            if (newCost < minCost)
                newPath = testPath;
                minCost = newCost;
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
    
    if (numValid > 0)
        isValidInsert = 1;
    else
        isValidInsert = 0;
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

function filepath = createPathPlot(dir, nodeTable, startNode, path, type, map) 
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
    
    outFileBase = strcat("pathOptimizerSolution-",string(datetime('now'),"yyyy-MM-dd-HH-mm-ss"));
    figPath = fullfile(dir,outFileBase);
    
    if ~exist(figPath,'dir')
       mkdir(figPath)
    end

    if isequal(figPath,0)
        clc
        disp('Solution plot save canceled.');
        return
    else
        fullFigPath = fullfile(figPath,outFileBase);
        saveas(solnFig,fullFigPath,'png');
        fullFigPath = strcat(fullFigPath,".png");
        
        filepath = figPath;
        
        winopen(fullFigPath);
    end
    
    mapPath = figPath;
    
    createSolnFile(dir,path,mapPath);
end

function filePath = createSolnFile(dir,path,mapPath)
    stopNum = [1:length(path(:,1))]';
    stopName = cell(length(path(:,1)),1);
    stopX = cell(length(path(:,1)),1);
    stopY = cell(length(path(:,1)),1);
    
    for ii = 1:length(path(:,1))
        stopName{ii,1} = path{ii,2};
        stopX{ii,1} = path{ii,3}(1);
        stopY{ii,1} = path{ii,3}(2);
    end
    
    outTable = table(stopNum,stopName,stopX,stopY,'VariableNames',{'Stop','Name','X','Y'});
    
    if (~isempty(mapPath))
        outFileBase = extractAfter(mapPath,(strlength(mapPath)-41));
        filePath = mapPath; 
    else
        outFileBase = strcat("pathOptimizerSolution-",string(datetime('now'),"yyyy-MM-dd-HH-mm-ss"));
        filePath = fullfile(dir,outFileBase);
    end
    
    if ~exist(filePath,'dir')
       mkdir(filePath)
    end

    if isequal(filePath,0)
        clc
        disp('Solution save canceled.');
        return
    else
        fullFilepath = fullfile(filePath,strcat(outFileBase,".csv"));
        writetable(outTable,fullFilepath);
    end
end

function pathOut = solver(nodeTable, startNode, type)
    lenCoords = length(nodeTable.Node(:));
%     loopLim = round(200-200*exp(-1*0.0277*lenCoords));
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
                unvisitedNodes{unvisitedItr,5} = nodeTable.Requires(tableIdx);
                
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
            path = {nodeTable.Node(startNode) nodeTable.Name(startNode) [nodeTable.X(startNode) nodeTable.Y(startNode)] nodeTable.Requires(startNode);nodeTable.Node(startNode) nodeTable.Name(startNode) [nodeTable.X(startNode) nodeTable.Y(startNode)] nodeTable.Requires(startNode)};
        else
            path = {nodeTable.Node(startNode) nodeTable.Name(startNode) [nodeTable.X(startNode) nodeTable.Y(startNode)] nodeTable.Requires(startNode)};
        end
        
        pathPrevious = path;
        
        allRepeatsVisited = 0;
        while (unvisitedLen > numRepeatable || ~allRepeatsVisited)
            randIdx = randi(unvisitedLen);
            randNode = unvisitedNodes(randIdx,[1:3 5]);
            isRepeatable = unvisitedNodes{randIdx,4};
            
            [path, newPathCost, insertStatus] = insertNode(path,randNode,type);
            
            if (~insertStatus)
                path = pathPrevious;
                continue
            else
                pathPrevious = path;
            end
            
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

function writePathOutput(path,type)
    clc

    if (type == "open")
        disp(['OPEN-LOOP SOLUTION: ' newline])
    else
        disp(['CLOSED-LOOP SOLUTION: ' newline])
    end
    
    for ii = 1:10
        disp(['Stop ' num2str(ii) ': ' char(path{ii,2}) ' [' num2str(path{ii,3}(1)) ', ' num2str(path{ii,3}(2)) ']'])
    end
end