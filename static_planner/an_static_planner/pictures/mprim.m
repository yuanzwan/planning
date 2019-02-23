%% This script generates motion primitives for path planning
% three motion primitives are generated:
% 1. proceed straight
% 2. change lane to the left
% 3. change lane to the right

fd = fopen('mprim.txt', 'w');
% define interval and velocity parameters
interval = 0.5; % [m]
velocity = 25; % [m/s]

% define number of motion primitives
numOfMotion = 3;
% each motion primitive contains the following information: start, end, length, cost,
% numOfElements and trajectory
motion = struct('start', [], 'end', [], 'length', [], 'cost', [], 'numOfElements', [], 'trajectory', []);
motions = repmat(motion, 1, numOfMotion);

% construct motion primitive one
motions(1).start = [0, 0, 0, 0];
motions(1).end = [10, 0, 0, 10/velocity];
motions(1).length = 10;
motions(1).cost = 10;
motions(1).numOfElements = ceil(motions(1).length / interval) + 1; 
motions(1).trajectory = zeros(motions(1).numOfElements, 5);
for i = 1:motions(1).numOfElements
    pos = motions(1).start(1) + (i - 1) * interval;
    motions(1).trajectory(i, :) = [pos, 0, 0, pos / velocity, velocity];
end

% lane changing motion primitive is generated using Bezier curve. Bezier curve ensures
% that the trajecotry is tangent at the beginning and the end. 
p0 = [0,0];
p3 = [100, 3.7];
% p1 and p2 are tuned based on the following requisites:
% 1. turning radius greater than 10m
% 2. lateral acceleration smaller than 3m/s^2
p1 = [40, 0];
p2 = [60, 3.7];
% pre-allocation for x[m], y[m] and theta[rad] and t[s]
size = ceil((p3(1) - p0(1) + p3(2) - p0(2)) / interval);
x = zeros(size, 1);
y = zeros(size, 1);
theta = zeros(size, 1);
t = zeros(size, 1);
% Bezier curve coefficients
BezierCoeff = [-p0 + 3*p1 - 3*p2 + p3; 3*p0 - 6*p1 + 3*p2; -3*p0 + 3*p1; p0];
BezierDiffCoeff = [-3*p0 + 9*p1 - 9*p2 + 3*p3; 6*p0 - 12*p1 + 6*p2; -3*p0 + 3*p1];
% inline function for calculating curvature
calCurvature = @(xA, yA, xB, yB, xC, yC) 2 * abs((xB - xA).* (yC - yA) - (xC - xA).* (yB - yA))./ ...
    sqrt(((xB - xA).^2 + (yB - yA).^2) * ((xC - xA).^2 + (yC - yA).^2) * ((xC - xB).^2 + (yC - yB).^2));

% generate intermediate points on the Bezier curve
s = 0;   
idx = 1; 
while (s < 1)
  sLow = s;
  sHigh = s + 0.1;
  idx = idx + 1;
  % use binary search to find interim points at 0.5m interval
  while (sLow < sHigh)
      sMid = sLow + 0.5 * (sHigh - sLow);
      p = [polyval(BezierCoeff(:, 1), sMid), polyval(BezierCoeff(:, 2), sMid)];
      dist = sqrt((p - [x(idx - 1), y(idx - 1)]) * (p - [x(idx - 1), y(idx - 1)])');
      if (abs(dist - interval) < 0.001)
          x(idx) = p(1);
          y(idx) = p(2);         
          % use derivative of bezier curve to calculate direction
          direction = [polyval(BezierDiffCoeff(:, 1), sMid), polyval(BezierDiffCoeff(:, 2), sMid)]; 
          theta(idx) = atan(direction(2) / direction(1));
          t(idx) = t(idx - 1) + dist / velocity;
          s = sMid;
          break;
      elseif (dist > interval)
          sHigh = sMid;
      else
          sLow = sMid;
      end
  end  
  % check if the turning curvature is satisfactory
  if (idx > 2)
      k = calCurvature(x(idx - 2), y(idx - 2), x(idx - 1), y(idx - 1), x(idx), y(idx));
      assert(k < 0.1,'turning radius smaller than 10m, passenger complaning');
      assert(k * velocity^2 < 3, 'lateral acceleration greater than 3m/s^2, passenger complaning');
  end
end

% set the end point
x(idx - 1) = p3(1);
y(idx - 1) = p3(2);
theta(idx - 1) = 0;
t(idx - 1) = t(idx - 2) + sqrt((p3 - [x(idx - 2), y(idx - 2)]) * (p3 - [x(idx - 2), y(idx - 2)])')/velocity;

% use the Bezier curve to construct motion primitive two
motions(2).start = [x(1), y(1), theta(1), t(1)];
motions(2).end = [x(idx - 1), y(idx - 1), theta(idx - 1), t(idx - 1)];
motions(2).length = t(idx - 1) * velocity;
motions(2).cost = 200; 
motions(2).numOfElements = idx - 1; 
motions(2).trajectory = [x(1:idx - 1), y(1:idx - 1), theta(1:idx - 1), t(1:idx - 1), velocity * ones(idx - 1, 1)];

% motion primitive three is symmetric to motion primitive two
motions(3).start = motions(2).start;
motions(3).end = [x(idx - 1), -y(idx - 1), -theta(idx - 1), t(idx - 1)];
motions(3).length = motions(2).length;
motions(3).cost = motions(2).cost;
motions(3).numOfElements = motions(2).numOfElements;
motions(3).trajectory = [x(1:idx - 1), -y(1:idx - 1), -theta(1:idx - 1), t(1:idx - 1), velocity * ones(idx - 1, 1)];

% plot the bezier curve
figure(1)
plot(motions(2).trajectory(:, 1), motions(2).trajectory(:, 2), '*');
xlabel('X (m)');
ylabel('Y (m)');
figure(2)
plot(motions(2).trajectory(:, 1), motions(2).trajectory(:, 3), '-');
xlabel('X (m)');
ylabel('\theta (radian)');
figure(3)
plot(motions(2).trajectory(:, 1), motions(2).trajectory(:, 4), 'o');
xlabel('X (m)');
ylabel('Time (s)');

% output motion primitive info to mprim.txt
fprintf(fd, '%d\n', numOfMotion);
for i = 1:3
    fprintf(fd, '%0.3f\t', motions(i).start, motions(i).end, motions(i).length,  motions(i).cost);
    fprintf(fd, '%d\n', motions(i).numOfElements);
    for j = 1:motions(i).numOfElements
       fprintf(fd, '%0.3f\t', motions(i).trajectory(j, 1:4));    
       fprintf(fd, '%0.3f\n', motions(i).trajectory(j, 5));       
    end
    fprintf(fd, '\n');
end

% close the file
fclose(fd);
