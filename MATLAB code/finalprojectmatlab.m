%% 
port     = "COM4";
baudrate = 115200;

% opening serial port
device = serialport(port, baudrate);
device.Timeout = 6000;
fprintf("Opening: %s\n", port);

pointsPerScan = 32;
numScans = 3;
N = pointsPerScan * numScans;   % 96 total points

flush(device);
fprintf("Waiting for MCU to finish all %d scans...\n", numScans);

% wait for MCU ready
while true
    if device.NumBytesAvailable > 0
        incomingChar = read(device, 1, "char");
        if incomingChar == 's'
            fprintf("Ready signal received from MCU!\n");
            break;
        end
    end
end

% Send acknowledgement to MCU to begin X data transmission
write(device, 's', "char");
fprintf("\nReceiving X coordinates:\n");

x = zeros(1, N);
for i = 1:N
    temp = readline(device);
    x(i) = str2double(temp);
    fprintf("%f\n", x(i));
end

fprintf("\nWaiting for Y coordinates...\n");

while true
    if device.NumBytesAvailable > 0
        incomingChar = read(device, 1, "char");
        if incomingChar == 's'
            fprintf("Ready signal received from MCU!\n");
            break;
        end
    end
end

% Send acknowledgement to MCU to begin Y data transmission
write(device, 's', "char");
fprintf("\nReceiving Y coordinates:\n");

y = zeros(1, N);
for i = 1:N
    temp = readline(device);
    y(i) = str2double(temp);
    fprintf("%f\n", y(i));
end

fprintf("\nClosing: %s\n", port);
clear device;

% reshape into [numScans x pointsPerScan]
x = reshape(x, pointsPerScan, numScans).';
y = reshape(y, pointsPerScan, numScans).';

% each scan is shifted 100 mm = 10 cm farther down the hallway
scanSpacing = 100; % mm
xOffset = (0:numScans-1) * scanSpacing;

fprintf("\nPlotting 3D data...\n");

figure;
hold on;

colors = lines(numScans);

% plot each YZ scan at a different X offset
for scan = 1:numScans
    
    Xplot = xOffset(scan) * ones(1, pointsPerScan); % hallway depth
    Yplot = y(scan, :);                             % vertical
    Zplot = x(scan, :);                             % horizontal

    scatter3(Xplot, Yplot, Zplot, 18, colors(scan,:), 'filled');
    plot3(Xplot, Yplot, Zplot, '-', 'Color', colors(scan,:), 'LineWidth', 1);
end

% connect corresponding points between scans
for i = 1:pointsPerScan
    plot3(xOffset, y(:, i), x(:, i), 'k-', 'LineWidth', 0.75);
end

% sensor start location
plot3(0, 0, 0, 'r+', 'MarkerSize', 12, 'LineWidth', 2);

grid on;
axis normal;

xlabel('X Offset / Hallway Depth (mm)');
ylabel('Y (Vertical Distance mm)');
zlabel('Z (Horizontal Distance mm)');
title('3 YZ Plane Hallway Scans (100 mm spacing)');

view(60, 10);
hold off;