clc

%%
filename = 'scan2.bag';
bag = rosbag(filename);

scanBag = select(bag, 'Topic', '/scan');
odomBag = select(bag, 'Topic', '/odom');

scanMsg = readMessages(scanBag);
odomMsg = readMessages(odomBag);

%% Make the Arrays
scan = zeros(length(scanMsg),640);
odom = zeros(length(odomMsg),3);

for i = 1:length(odomMsg)
    odom(i,1) = odomMsg{i,1}.Pose.Pose.Position.X;
    odom(i,2) = odomMsg{i,1}.Pose.Pose.Position.Y;
    
    z = odomMsg{i,1}.Pose.Pose.Orientation.Z;
    w = odomMsg{i,1}.Pose.Pose.Orientation.W;
    odom(i,3) = atan2(2*(w*z), 1-2*z^2);
end

for col=1:length(scanMsg)
    for row=1:640
        if ~(isnan(scanMsg{col,1}.Ranges(row)))
            scan(col,row) = scanMsg{col,1}.Ranges(row);
        end
    end
end

%% Look at the arrays
clc;
for col=690:700
    for row=1:640
        dz = scan(col, row) - scan(col+1, row);
        if abs(dz) > 0.25
            fprintf('Bad reading: (%d, %d) --> %.4f, %.4f\n',col,row, scan(col, row),scan(col+1, row))
        end
    end
end