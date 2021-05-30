% A simple priority queue that takes arrays as inputs and uses a specific
% column of the 1xN arrays to sort by the minimum value.
%
% Utilizes a minheap to ensure quick operation even with queues with 
% > 100,000 elements 
%
% Andrew Woodward - Fall 2018
%
%   Example uses:
%                   q = PriorityQueue(1);
%                   q.insert([1 2 3 4]);
%                   q.insert([3 4 2 3 5]);
%                   q.insert([5 2 1]);
%                   disp('first element');
%                   disp(q.peek());
%                   disp('contains?');
%                   disp(q.contains([3 4 2 3 5]));
%                   disp('size before and after remove');
%                   disp(q.size());
%                   q.remove();
%                   disp(q.size());
%                   q.remove([5 2 1]);
%                   q.clear();
%

classdef PriorityQueue < handle
    properties (Access = private)
        Data % the data in the queue
        Size % the size of the queue
        Column % the comparator column of the data
    end
    methods
        % contstuctor of the queue
        function obj = PriorityQueue(varargin)
            obj.Size = 0;
            obj.Data = {};
            if size(varargin) == 1 
                obj.Column = varargin{1};
            else
                obj.Column = 1; % default comparator column is the first
            end
        end
        
        % insert an array into the queue given by arg1
        function insert(obj, arg1)
            if size(arg1,2) < obj.Column
                disp('error:insert array too small for queue');
                return
            end
            obj.Size = obj.Size + 1;
            obj.Data{obj.Size} = arg1; % add new element to bottom of heap
            parentIter = floor(obj.Size/2);
            currentIter = obj.Size;
            % perform bubble up operation
            while parentIter > 0
                if obj.Data{currentIter}(obj.Column) < obj.Data{parentIter}(obj.Column)
                    % the current element is smaller than parent so swap
                    temp = obj.Data{currentIter};
                    obj.Data{currentIter} = obj.Data{parentIter};
                    obj.Data{parentIter} = temp;
                    currentIter = parentIter;
                    parentIter = floor(currentIter/2);
                else
                    break;
                end
            end
        end
        
        % remove and return the top array from queue or any that match the input array
        function node = remove(obj, varargin)
            node = 0;
            if obj.Size == 0
                disp('remove from an empty queue, returing 0');
                return
            end
            if size(varargin,2) > 0
                for i=1:obj.Size
                    if isequal(obj.Data{i}, varargin{1})
                        node = obj.Data{i};
                        obj.Data{i} = [];
                        obj.Size = obj.Size - 1;
                    end
                end
                obj.Data = obj.Data(~cellfun('isempty',obj.Data)); % remove empty element from cell array
            else
                node = obj.Data{1};
                obj.Data{1} = obj.Data{obj.Size}; % replace root with last element
                obj.Data{obj.Size} = 0; % clear the last element
                obj.Size = obj.Size - 1;
                leftChild = 2;
                rightChild = 3;
                currentIter = 1;
                % perform bubble down
                while currentIter < obj.Size
                    if leftChild <= obj.Size && rightChild <= obj.Size && (obj.Data{currentIter}(1,obj.Column) > obj.Data{leftChild}(1,obj.Column) || obj.Data{currentIter}(1,obj.Column) > obj.Data{rightChild}(1,obj.Column))
                        if obj.Data{leftChild}(1,obj.Column) < obj.Data{rightChild}(1,obj.Column)
                            % left child is smaller
                            tmp = obj.Data{currentIter};
                            obj.Data{currentIter} = obj.Data{leftChild};
                            obj.Data{leftChild} = tmp;
                            currentIter = leftChild;
                            leftChild = currentIter*2;
                            rightChild = currentIter*2+1;
                        else
                            % right child is smaller
                            tmp = obj.Data{currentIter};
                            obj.Data{currentIter} = obj.Data{rightChild};
                            obj.Data{rightChild} = tmp;
                            currentIter = rightChild;
                            leftChild = currentIter*2;
                            rightChild = currentIter*2+1;
                        end
                    elseif leftChild <= obj.Size && rightChild > obj.Size && obj.Data{currentIter}(1,obj.Column) > obj.Data{leftChild}(1,obj.Column)
                        % only the left child exists
                        tmp = obj.Data{currentIter};
                        obj.Data{currentIter} = obj.Data{leftChild};
                        obj.Data{leftChild} = tmp;
                        break;
                    else
                        % either the children are empty or the currentIter value is minimum
                        break;
                    end
                end
            end
        end
        
        % peek and return the first element of the queue
        function node = peek(obj)
            if obj.Size == 0
                node = 0;
                disp('peek from an empty queue, returing 0');
                return
            end
            node = obj.Data{1}; % returns the first element in queue
        end
        
        % returns the size of the queue
        function sz = size(obj)
            sz = obj.Size; % returns the size of the queue
        end
        
        % clears the entire queue
        function clear(obj)
            obj.Data = {};
            obj.Size = 0;
        end
        
        % checks if the queue contains a specific array
        function flag = contains(obj, array)
            flag = 0;
            for i=1:obj.Size
                if isequal(obj.Data{i}, array)
                    flag = 1; % if the array exists in the queue
                    return
                end
            end
        end
        
        % returns all the elements of the queue as a cell array
        function queue = elements(obj)
            queue = obj.Data;
        end
        
    end
end