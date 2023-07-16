function RoutePlan::NURBSS(pose, NURBSDATA)  
if Time_Limit  
    try  
        if size(pose) == [0,0]  
            throw "input data is error!"  
        else  
            NURBSDATA.clear();  
            row = size(pose,1);  
            D_X = zeros(row,1);  
            D_Y = zeros(row,1);  
            D_Z = zeros(row,1);  
            U = zeros(row+4,1);  
            P_X = zeros(row,1);  
            P_Y = zeros(row,1);  
            P_Z = zeros(row,1);  
            W = zeros(row,1);  
              
            p = zeros(6,1);  
            V_T = zeros(3,1);  
              
            Tdd = [];  
            s_m = [];  
            s = [];  
            Sr_m = [];  
            Sr = [];  
            u = [];  
            Ts = [];  
              
            for i = 1:row  
                D_X(i) = 1000 * pose(i,1);  
                D_Y(i) = 1000 * pose(i,2);  
                D_Z(i) = 1000 * pose(i,3);  
            end  
              
            counter_BA(D_X, D_Y, D_Z, row, P_X, P_Y, P_Z, U, W);  
              
            ucount = 0;  
            pro = 0.001;  
            for i = 0:(1/pro)-1  
                u(end+1) = ucount;  
                ucount = ucount + pro;  
            end  
              
            n = size(u,1);  
            m = row;  
            arcs;  
            s_m(1) = 0;  
            for i = 0:(n-1)  
                arcs = ComputeArcLengthOfBspline(row, P_X, P_Y, P_Z, U, u(i), u(i+1)); %计算样条曲线弧长  
                s_m(end+1) = s_m(i) + arcs;  
            end  
            St = s_m(end);  
            for i = 0:(size(s_m,1))  
                s(end+1) = s_m(i) / St;  
                s_m.clear;  
                ret = polyFit(s, u, 5, p);  
                % TODO: add relevant code here  
            end  
        end  
    catch exception  
        % handle the exception here  
    end  
end