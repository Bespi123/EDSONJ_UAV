% 1. Define Sample Waypoints (x  = East, y = North)
waypoints_x = [0, 3, 5, 8, 10, 12]; % East coordinates in meters
waypoints_y = [0, 1, 4, 5,  3,  4]; % North coordinates in meters

samples_number = 200;

[path_x_fn, path_y_fn, deriv_x_fn, deriv_y_fn, total_s] = ...
    parameterize_path_by_theta(waypoints_x, waypoints_y, samples_number);

% Store path info
path_params.path_x_fn = path_x_fn;
path_params.path_y_fn = path_y_fn;
path_params.deriv_x_fn = deriv_x_fn; % Corresponds to getting xp'(theta)
path_params.deriv_y_fn = deriv_y_fn; % Corresponds to getting yp'(theta)
path_params.total_s = total_s;


final_distance = sqrt(waypoints_x(end)^2+waypoints_y(end)^2);
surge_vel = 2.5;   % Constant surge velocity 2.5m/s
time = linspace(0,final_distance/surge_vel, samples_number); 
% Ensure waypoints are ordered by x for standard spline function (if needed, although spline handles it)
% [waypoints_x_sorted, order] = sort(waypoints_x);
% waypoints_y_sorted = waypoints_y(order);
% Using original order assuming x is monotonically increasing for simplicity here.

% 2. Generate points along the spline for smooth plotting
% Create a dense range of x values between the first and last waypoint
x_smooth = linspace(min(waypoints_x), max(waypoints_x), samples_number); 

% 3. Calculate the corresponding y values using cubic spline interpolation
% The 'spline' function calculates the piecewise polynomial and evaluates it
y_smooth = spline(waypoints_x, waypoints_y, x_smooth);

% 4. Obtain the gamma_p angle relative to the Nort
gamma = [atan2(diff(y_smooth),diff(x_smooth)), atan2(diff(y_smooth(end-1:end)),diff(x_smooth(end-1:end)))];


% 4. Plot the trajectory
figure; % Create a new figure window
hold on; % Allow multiple plots on the same axes

% Plot the smooth trajectory
theta_plot = linspace(0, total_s, 200);
xpath_plot = ppval(path_x_fn, theta_plot);
ypath_plot = ppval(path_y_fn, theta_plot);
plot(xpath_plot, ypath_plot, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Desired Path');

% Plot the original waypoints
plot(waypoints_x, waypoints_y, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'Waypoints');

xlabel('East');
ylabel('North');


function [path_x_spline, path_y_spline, deriv_x_spline, deriv_y_spline, total_s] = ...
    parameterize_path_by_theta(wp_x, wp_y, num_initial_points)
    % Parameterizes the path defined by waypoints (wp_x, wp_y) by arc length (theta).
    % Returns spline functions (ppform) for x(thetha), y(thetha), dx/d(thetha), dy/d(thetha), and the total path length.
    %
    % INPUTS:
    %   wp_x: Vector of East coordinates for the waypoints.
    %   wp_y: Vector of North coordinates for the waypoints.
    %
    % OUTPUTS:
    %   path_x_spline: Spline structure (ppform) representing xp(thetha). Use ppval(path_x_spline, thetha) to evaluate.
    %   path_y_spline: Spline structure (ppform) representing yp(thetha). Use ppval(path_y_spline, thetha) to evaluate.
    %   deriv_x_spline: Spline structure (ppform) representing dxp/ds (xp'(thetha)). Use ppval(deriv_x_spline, thetha).
    %   deriv_y_spline: Spline structure (ppform) representing dyp/ds (yp'(thetha)). Use ppval(deriv_y_spline, thetha).
    %   total_s: The total calculated arc length of the path.

    % Step A: Create an initial fine-grained path using spline based on x-coordinate
    % This provides a dense set of (x,y) points representing the path geometry needed for arc length calculation.
    % It assumes x is mostly monotonic; if path doubles back strongly in x, this initial step might need refinement.
    %num_initial_points = length(wp_x) * 50; % Number of points for initial sampling (adjust for accuracy vs speed)
   
    x_fine = linspace(min(wp_x), max(wp_x), num_initial_points); % Generate dense x-coordinates
    y_fine = spline(wp_x, wp_y, x_fine); % Calculate corresponding y-coordinates on the initial smooth path

    % Step B: Calculate segment lengths and cumulative arc length 's'
    % Calculate differences between consecutive points
    dx = diff(x_fine); % Difference in x between points i+1 and i
    dy = diff(y_fine); % Difference in y between points i+1 and i

    % Calculate the Euclidean distance (length) of each small segment
    segment_lengths = sqrt(dx.^2 + dy.^2);

    % Calculate the cumulative sum of segment lengths to get arc length from the start
    % s_cumulative(i) is the arc length to point i on the fine path.
    s_cumulative = [0, cumsum(segment_lengths)]; % Add 0 for the starting point

    % Store the total arc length of the path
    total_s = s_cumulative(end);

    % Step C: Create new splines parameterized by arc length 's' (our path parameter theta)
    % We now want functions x(s) and y(s). The known data points are (s_cumulative, x_fine) and (s_cumulative, y_fine).

    % The 'spline' function requires the parameter vector (s_cumulative here) to be strictly monotonic.
    % Use 'unique' to remove any duplicate arc length values that might arise from zero-length segments
    % (due to numerical precision or coincident points in the fine sampling). 'stable' preserves order.
    % 'ia' gives the indices of the first occurrence of each unique value in s_cumulative.
    [s_unique, ia, ~] = unique(s_cumulative, 'stable');

    % Select the corresponding x and y coordinates for the unique arc length values
    x_for_s_spline = x_fine(ia);
    y_for_s_spline = y_fine(ia);

    % Check if we have enough unique points to create a valid spline
    if length(s_unique) < 2
        error('Not enough unique points to create arc-length parameterized spline. Check waypoints or initial sampling density.');
    end

    % Create the cubic spline for x coordinate as a function of arc length s: xp(s)
    path_x_spline = spline(s_unique, x_for_s_spline); % Represents xp(theta) where theta=s

    % Create the cubic spline for y coordinate as a function of arc length s: yp(s)
    path_y_spline = spline(s_unique, y_for_s_spline); % Represents yp(theta) where theta=s

    % Step D: Get splines for the derivatives dx/ds and dy/ds (or xp'(theta), yp'(theta))
    % These are needed for calculating the path tangential angle gamma_p (Eq. 1).
    % 'fnder' computes the derivative of a spline given in ppform. The '1' indicates the first derivative.
    deriv_x_spline = fnder(path_x_spline, 1); % Represents dxp/ds = xp'(theta)
    deriv_y_spline = fnder(path_y_spline, 1); % Represents dyp/ds = yp'(theta)
end

function [theta_star, x_p, y_p] = find_orthogonal_projection(veh_x, veh_y, path_params, initial_theta_guess)
    % Finds the path parameter theta_star such that the vector from the path 
    % point (xp(theta_star), yp(theta_star)) to the vehicle (veh_x, veh_y)
    % is orthogonal to the path tangent at theta_star.
    % This aims to satisfy the geometric condition of Eq. (3) from the Fossen & Pettersen paper.
    %
    % INPUTS:
    %   veh_x, veh_y: Current East and North coordinates of the vehicle.
    %   path_params: Structure containing:
    %       path_x_fn: Spline function for path x-coordinate vs. arc length (xp(s))
    %       path_y_fn: Spline function for path y-coordinate vs. arc length (yp(s))
    %       deriv_x_fn: Spline function for path x-derivative vs. arc length (xp'(s))
    %       deriv_y_fn: Spline function for path y-derivative vs. arc length (yp'(s))
    %       total_s: Total arc length of the path.
    %   initial_theta_guess: An initial guess for theta_star, typically the previous value.
    %
    % OUTPUTS:
    %   theta_star: The arc length parameter of the orthogonal projection point.
    %   x_p, y_p: The East and North coordinates of the orthogonal projection point.

    % Unpack path parameters from the structure for easier access
    path_x_fn = path_params.path_x_fn;
    path_y_fn = path_params.path_y_fn;
    deriv_x_fn = path_params.deriv_x_fn;
    deriv_y_fn = path_params.deriv_y_fn;
    total_s = path_params.total_s;

    % Define the function whose root implies orthogonality.
    % Let V be the vector from path point to vehicle: V = [veh_x - xp(theta); veh_y - yp(theta)]
    % Let T be the path tangent vector: T = [xp'(theta); yp'(theta)]
    % Orthogonality means the dot product V . T = 0.
    % So, h(theta) = (veh_x - xp(theta))*xp'(theta) + (veh_y - yp(theta))*yp'(theta)
    % We want to find theta such that h(theta) = 0.
    orthogonality_condition = @(theta_val) ...
        (veh_x - ppval(path_x_fn, theta_val)) * ppval(deriv_x_fn, theta_val) + ...
        (veh_y - ppval(path_y_fn, theta_val)) * ppval(deriv_y_fn, theta_val);

    % Set options for the fzero root-finding function:
    % TolX: Tolerance on the solution theta_val
    % Display 'off': Suppress fzero's iterative display
    fzero_options = optimset('TolX', 1e-6, 'Display', 'off');
    
    % Clamp the initial_theta_guess to be within the valid path parameter range [0, total_s]
    % This helps fzero and prevents errors if the guess is outside the domain.
    theta_guess_clamped = max(0, min(initial_theta_guess, total_s));

    % Use a try-catch block to handle potential errors from fzero
    try
        % Attempt to find the root (theta_star) of the orthogonality_condition function
        % using fzero. fzero searches for a point where the function value is zero.
        % It starts its search near theta_guess_clamped.
        %
        % For fzero to work robustly, especially with a single guess point,
        % the function should ideally cross zero near that guess.
        % An alternative fzero syntax uses an interval [a,b] where f(a) and f(b) have opposite signs.
        % The code block below tries to provide a small interval around the guess if a sign change is detected.
        
        % Heuristic for search interval around the guess (can be refined)
        search_interval_radius = total_s * 0.1; % Search within 10% of path length around guess
        lower_bound = max(0, theta_guess_clamped - search_interval_radius);
        upper_bound = min(total_s, theta_guess_clamped + search_interval_radius);
        
        % Check if the function changes sign within the heuristic interval
        if orthogonality_condition(lower_bound) * orthogonality_condition(upper_bound) <= 0
             theta_star = fzero(orthogonality_condition, [lower_bound, upper_bound], fzero_options);
        else
            % If no sign change is detected in the interval, try with the single point guess.
            % This might fail if the guess is not close enough or if there are flat regions/multiple roots.
            theta_star = fzero(orthogonality_condition, theta_guess_clamped, fzero_options);
        end
    catch ME
        % If fzero fails (e.g., cannot find a root, complex values, NaN, etc.),
        % issue a warning and fall back to the simpler (but less accurate) 
        % method of finding the geometrically closest point using a grid search.
        %warning('fzero failed to find orthogonal projection: %s. Falling back to simple closest point search.', ME.message);
        
        % Fallback: Simple closest point search
        num_search_points_fallback = 200; % Number of points for fallback search
        theta_search_fb = linspace(0, total_s, num_search_points_fallback);
        path_points_x_fb = ppval(path_x_fn, theta_search_fb);
        path_points_y_fb = ppval(path_y_fn, theta_search_fb);
        
        % Calculate squared distances to vehicle
        distances_sq_fb = (path_points_x_fb - veh_x).^2 + (path_points_y_fb - veh_y).^2;
        
        % Find the index of the minimum distance
        [~, idx_closest_fb] = min(distances_sq_fb);
        
        % The path parameter corresponding to the closest point
        theta_star = theta_search_fb(idx_closest_fb);
    end
    
    % Ensure the final theta_star is strictly within the path bounds [0, total_s],
    % as fzero might sometimes return values slightly outside due to tolerances.
    theta_star = max(0, min(theta_star, total_s));

    % Get the (x,y) coordinates of this projected point on the path
    x_p = ppval(path_x_fn, theta_star); % x-coordinate of the point on path at theta_star
    y_p = ppval(path_y_fn, theta_star); % y-coordinate of the point on path at theta_star
end