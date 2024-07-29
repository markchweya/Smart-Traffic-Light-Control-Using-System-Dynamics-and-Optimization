%% Traffic Light Control Simulation and Optimization
% This script simulates traffic light control using fixed and optimized timings.

% Main Script Section

% Define parameters
arrival_rates = [500, 450, 480, 520]; % vehicles per hour
vehicle_speeds = [30, 35, 32, 28]; % km/h
initial_queue_lengths = [100, 120, 110, 130]; % meters
fixed_traffic_light_timings = [30, 5, 35;  % Direction 1: green, yellow, red durations (seconds)
                               25, 5, 30;  % Direction 2
                               28, 5, 32;  % Direction 3
                               27, 5, 33]; % Direction 4
vehicle_length = 5; % average vehicle length in meters
duration = 3600; % simulation duration in seconds

% Convert arrival rates from vehicles per hour to vehicles per second
arrival_rates_per_sec = arrival_rates / 3600;

%% Baseline Scenario
% Solve the differential equations using ode45 for the baseline scenario
[t, q_baseline] = ode45(@(t, q) queue_dynamics(t, q, arrival_rates_per_sec, vehicle_speeds, fixed_traffic_light_timings, vehicle_length), [0 duration], initial_queue_lengths);

% Calculate average waiting times and vehicle capacities
average_waiting_times_baseline = calculate_waiting_times(q_baseline, arrival_rates_per_sec');
vehicle_capacities_baseline = calculate_vehicle_capacities(vehicle_speeds, fixed_traffic_light_timings, vehicle_length);

% Plot the queue lengths for the baseline scenario
figure;
for i = 1:length(arrival_rates)
    subplot(2, 2, i);
    plot(t, q_baseline(:, i));
    title(['Queue Length for Direction ' num2str(i) ' (Baseline)']);
    xlabel('Time (seconds)');
    ylabel('Queue Length (meters)');
end

% Plot the average waiting times for the baseline scenario
figure;
for i = 1:length(arrival_rates)
    subplot(2, 2, i);
    plot(t, average_waiting_times_baseline(:, i));
    title(['Average Waiting Time for Direction ' num2str(i) ' (Baseline)']);
    xlabel('Time (seconds)');
    ylabel('Waiting Time (seconds)');
end

% Display vehicle capacities and fixed traffic light timings
disp('Baseline Scenario:');
disp('Vehicle Capacities (vehicles per second):');
disp(vehicle_capacities_baseline);
disp('Fixed Traffic Light Timings (seconds):');
disp(fixed_traffic_light_timings);

%% Optimization using Genetic Algorithm
% Define bounds for traffic light timings (green, yellow, red)
lb = repmat([10, 3, 10], 1, 4); % lower bounds for [green, yellow, red] durations for each direction
ub = repmat([60, 5, 60], 1, 4); % upper bounds for [green, yellow, red] durations for each direction

% Define the objective function handle
objective_function_handle = @(timings) objective_function(timings, arrival_rates, vehicle_speeds, initial_queue_lengths, vehicle_length, duration);

% Set up Genetic Algorithm parameters
options = optimoptions('ga', 'Display', 'iter', 'PopulationSize', 50, 'MaxGenerations', 100, 'CrossoverFraction', 0.8);

% Run the Genetic Algorithm
[optimal_timings, optimal_avg_waiting_time] = ga(objective_function_handle, 12, [], [], [], [], lb, ub, [], options);

% Reshape the optimal timings to match the traffic light structure
optimal_traffic_light_timings = reshape(optimal_timings, [4, 3]);

% Display the optimal traffic light timings and the corresponding average waiting time
disp('Optimized Scenario:');
disp('Optimal Traffic Light Timings (seconds):');
disp(optimal_traffic_light_timings);
disp(['Optimal Average Waiting Time: ' num2str(optimal_avg_waiting_time) ' seconds']);

% Validate the results with the optimal timings
[t, q_optimized] = ode45(@(t, q) queue_dynamics(t, q, arrival_rates_per_sec, vehicle_speeds, optimal_traffic_light_timings, vehicle_length), [0 duration], initial_queue_lengths);

% Calculate average waiting times and vehicle capacities
average_waiting_times_optimized = calculate_waiting_times(q_optimized, arrival_rates_per_sec');
vehicle_capacities_optimized = calculate_vehicle_capacities(vehicle_speeds, optimal_traffic_light_timings, vehicle_length);

% Plot the queue lengths for the optimized scenario
figure;
for i = 1:length(arrival_rates)
    subplot(2, 2, i);
    plot(t, q_optimized(:, i));
    title(['Queue Length for Direction ' num2str(i) ' (Optimized)']);
    xlabel('Time (seconds)');
    ylabel('Queue Length (meters)');
end

% Plot the average waiting times for the optimized scenario
figure;
for i = 1:length(arrival_rates)
    subplot(2, 2, i);
    plot(t, average_waiting_times_optimized(:, i));
    title(['Average Waiting Time for Direction ' num2str(i) ' (Optimized)']);
    xlabel('Time (seconds)');
    ylabel('Waiting Time (seconds)');
end

% Display vehicle capacities and optimal traffic light timings
disp('Vehicle Capacities (vehicles per second):');
disp(vehicle_capacities_optimized);
disp('Optimal Traffic Light Timings (seconds):');
disp(optimal_traffic_light_timings);

%% Analysis and Comparison
% Calculate average queue lengths over the simulation period
avg_queue_length_baseline = mean(q_baseline);
avg_queue_length_optimized = mean(q_optimized);

% Display average queue lengths
disp('Average Queue Lengths (Baseline):');
disp(avg_queue_length_baseline);
disp('Average Queue Lengths (Optimized):');
disp(avg_queue_length_optimized);

% Calculate average waiting times over the simulation period
avg_waiting_time_baseline = mean(average_waiting_times_baseline);
avg_waiting_time_optimized = mean(average_waiting_times_optimized);

% Display average waiting times
disp('Average Waiting Times (Baseline):');
disp(avg_waiting_time_baseline);
disp('Average Waiting Times (Optimized):');
disp(avg_waiting_time_optimized);

% Calculate and display the percentage reduction in average waiting times
reduction_in_waiting_time = ((avg_waiting_time_baseline - avg_waiting_time_optimized) ./ avg_waiting_time_baseline) * 100;
disp('Percentage Reduction in Average Waiting Times:');
disp(reduction_in_waiting_time);

% Calculate and display the percentage reduction in average queue lengths
reduction_in_queue_length = ((avg_queue_length_baseline - avg_queue_length_optimized) ./ avg_queue_length_baseline) * 100;
disp('Percentage Reduction in Average Queue Lengths:');
disp(reduction_in_queue_length);

% Plot the comparison of average queue lengths for each direction
figure;
bar([avg_queue_length_baseline' avg_queue_length_optimized'], 'grouped');
title('Comparison of Average Queue Lengths');
xlabel('Direction');
ylabel('Average Queue Length (meters)');
legend('Baseline', 'Optimized');

% Plot the comparison of average waiting times for each direction
figure;
bar([avg_waiting_time_baseline' avg_waiting_time_optimized'], 'grouped');
title('Comparison of Average Waiting Times');
xlabel('Direction');
ylabel('Average Waiting Time (seconds)');
legend('Baseline', 'Optimized');

%% Function Definitions

% Function to define system dynamics for queue lengths
function dqdt = queue_dynamics(t, q, arrival_rates, vehicle_speeds, traffic_light_timings, vehicle_length)
    num_directions = length(arrival_rates);
    dqdt = zeros(num_directions, 1);
    
    for i = 1:num_directions
        green_time = traffic_light_timings(i, 1);
        red_time = sum(traffic_light_timings(i, 2:end));
        cycle_time = green_time + red_time;
        
        % Service rate calculation
        if mod(t, cycle_time) <= green_time
            service_rate = (vehicle_speeds(i) * green_time) / vehicle_length;
        else
            service_rate = 0;
        end
        
        % Differential equation for queue length
        dqdt(i) = arrival_rates(i) - service_rate;
    end
end

% Function to calculate average waiting times
function waiting_times = calculate_waiting_times(queue_lengths, arrival_rates)
    waiting_times = queue_lengths ./ arrival_rates;
end

% Function to calculate vehicle capacities
function capacities = calculate_vehicle_capacities(vehicle_speeds, traffic_light_timings, vehicle_length)
    green_times = traffic_light_timings(:, 1);
    capacities = (vehicle_speeds .* green_times) / vehicle_length;
end

% Objective function for optimization
function avg_waiting_time = objective_function(timings, arrival_rates, vehicle_speeds, initial_queue_lengths, vehicle_length, duration)
    % Reshape timings to match the traffic light structure
    traffic_light_timings = reshape(timings, [4, 3]);
    
    % Convert arrival rates from vehicles per hour to vehicles per second
    arrival_rates_per_sec = arrival_rates / 3600;
    
    % Solve the differential equations using ode45
    [t, q] = ode45(@(t, q) queue_dynamics(t, q, arrival_rates_per_sec, vehicle_speeds, traffic_light_timings, vehicle_length), [0 duration], initial_queue_lengths);
    
    % Calculate average waiting times
    waiting_times = calculate_waiting_times(q, arrival_rates_per_sec');
    
    % Calculate the average waiting time over the simulation period
    avg_waiting_time = mean(mean(waiting_times));
end
