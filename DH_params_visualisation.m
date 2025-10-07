clc;
clear all;
close all;

% --- DEPENDENCY CHECK ---
if ~license('test', 'Symbolic_Toolbox')
    error('Symbolic Math Toolbox is required for this script to run.');
end

% --- USER INPUT SECTION ---
fprintf('Denavit-Hartenberg (DH) Transformation Calculator\n');
fprintf('-------------------------------------------------\n');
while true
    dof = input('Enter the number of Degrees of Freedom (DOF) for the robot: ');
    if isnumeric(dof) && isscalar(dof) && dof > 0 && floor(dof) == dof
        break;
    else
        disp('Invalid input. Please enter a single positive integer.');
    end
end

% MODIFIED: Use two tables - one for symbolic input, one for numeric plotting values
dh_table_sym = cell(dof, 4);
dh_table_num = zeros(dof, 4);

fprintf('\nEnter the DH parameters for each joint.\n');
fprintf('You can enter numbers (e.g., 90) or symbolic variables (e.g., th1).\n');
fprintf('Angles (theta, alpha) should be in DEGREES if entered as numbers.\n\n');

% MODIFIED: Enhanced input loop to handle both numbers and symbols
for i = 1:dof
    fprintf('--- For Joint %d ---\n', i);
    
    % Parameter 1: theta
    [dh_table_sym{i, 1}, dh_table_num(i, 1)] = process_input(sprintf('Enter theta_%d: ', i));
    % Parameter 2: d
    [dh_table_sym{i, 2}, dh_table_num(i, 2)] = process_input(sprintf('Enter d_%d: ', i));
    % Parameter 3: a
    [dh_table_sym{i, 3}, dh_table_num(i, 3)] = process_input(sprintf('Enter a_%d: ', i));
    % Parameter 4: alpha
    [dh_table_sym{i, 4}, dh_table_num(i, 4)] = process_input(sprintf('Enter alpha_%d: ', i));
    
    fprintf('\n');
end

disp('Completed DH Parameter Table (for Plotting):');
disp('   theta      d         a       alpha');
disp('-----------------------------------------');
disp(dh_table_num);

% --- MATRIX GENERATION ---
A_num = cell(1, dof); 
A_sym = cell(1, dof); 

% MODIFIED: Matrix generation now builds from the two separate tables
for i = 1:dof
    % 1. Create the numeric matrix for plotting using the numeric table
    A_num{i} = create_A_matrix_numeric(dh_table_num(i,:));

    % 2. Create the symbolic matrix using the symbolic table
    % For any parameter that was entered as a number, it needs to be converted to radians
    % before being used in the symbolic trigonometric functions.
    [theta_s, is_th_var] = convert_to_sym(dh_table_sym{i, 1});
    [d_s, ~]             = convert_to_sym(dh_table_sym{i, 2});
    [a_s, ~]             = convert_to_sym(dh_table_sym{i, 3});
    [alpha_s, is_al_var] = convert_to_sym(dh_table_sym{i, 4});

    if ~is_th_var % If user entered a number like '90', convert it to radians for symbolic math
        theta_s = deg2rad(theta_s);
    end
    if ~is_al_var % If user entered a number like '-90', convert it to radians
        alpha_s = deg2rad(alpha_s);
    end
    
    A_sym{i} = create_A_matrix_symbolic(theta_s, d_s, a_s, alpha_s);
end

% --- 3D VISUALIZATION ---
fprintf('\nGenerating Link Coordinate Diagram (using numerical values)...\n');
figure;
axis equal; grid on; xlabel('X'); ylabel('Y'); zlabel('Z');
view(3); hold on; title('Robot Link Coordinate Frames (DH Convention)');
axis([-50 50 -50 50 -50 50]);

T_previous = eye(4); 
plotCoordinateFrame(T_previous, '0'); 

for i = 1:dof
    T_current = T_previous * A_num{i}; % T0_i = T0_{i-1} * A_i

    origin_previous = T_previous(1:3, 4);
    origin_current  = T_current(1:3, 4);

    plot3([origin_previous(1) origin_current(1)], ...
          [origin_previous(2) origin_current(2)], ...
          [origin_previous(3) origin_current(3)], ...
          'k', 'LineWidth', 2.5); 

    plotCoordinateFrame(T_current, num2str(i));

    T_previous = T_current;
end

drawnow;

fprintf('Diagram generated. You can now rotate and inspect the figure with your mouse.\n');
input('Press Enter in the Command Window to continue to the next step...');

% --- AUTOMATED TRANSFORMATION REPRESENTATION SECTION ---
% This section remains unchanged as it correctly uses the A_sym and A_num cells.
fprintf('\n=========================================================\n');
fprintf('   COMPOSITE TRANSFORMATION REPRESENTATION STEPS\n');
fprintf('=========================================================\n');

% --- Step 1: Transformation from Wrist (Frame N-1) to Base (Frame 0) ---
if dof >= 2
    fprintf('\n--- 1. Transformation from Wrist (Frame %d) to Base (Frame 0) ---\n', dof-1);
    T_wrist_base_sym = sym(eye(4));
    for i = 1:(dof-1)
        T_wrist_base_sym = T_wrist_base_sym * A_sym{i};
    end
    fprintf('\nSymbolic Representation (T_wrist_to_base) in cos/sin form:\n');
    disp(simplify(T_wrist_base_sym));

    T_wrist_base_num = eye(4);
    for i = 1:(dof-1)
        T_wrist_base_num = T_wrist_base_num * A_num{i};
    end
    fprintf('\nNumerical Result (substituting DH table values):\n');
    disp(T_wrist_base_num);
else
    fprintf('\n--- 1. Wrist-to-Base transformation not applicable (DOF < 2) ---\n');
end

% --- Step 2: Transformation from Tool (Frame N) to Wrist (Frame N-1) ---
if dof >= 1
    fprintf('\n--- 2. Transformation from Tool (Frame %d) to Wrist (Frame %d) ---\n', dof, dof-1);
    fprintf('\nSymbolic Representation (T_tool_to_wrist) in cos/sin form:\n');
    disp(A_sym{dof});
    fprintf('\nNumerical Result (substituting DH table values):\n');
    disp(A_num{dof});
else
     fprintf('\n--- 2. Tool-to-Wrist transformation not applicable (DOF < 1) ---\n');
end

% --- Step 3: Transformation from Tool (Frame N) to Base (Frame 0) ---
if dof >= 1
    fprintf('\n--- 3. Final Transformation from Tool (Frame %d) to Base (Frame 0) ---\n', dof);
    T_tool_base_sym = sym(eye(4));
    for i = 1:dof
        T_tool_base_sym = T_tool_base_sym * A_sym{i};
    end
    fprintf('\nSymbolic Representation (T_tool_to_base) in cos/sin form:\n');
    disp(simplify(T_tool_base_sym));
    
    T_tool_base_num = eye(4);
    for i = 1:dof
        T_tool_base_num = T_tool_base_num * A_num{i};
    end
    fprintf('\nNumerical Result (substituting DH table values):\n');
    disp(T_tool_base_num);
else
    fprintf('\n--- 3. Tool-to-Base transformation not applicable (DOF < 1) ---\n');
end

fprintf('\n=========================================================\n');
fprintf('                      END OF ANALYSIS\n');
fprintf('=========================================================\n');


% --- HELPER FUNCTIONS ---

% NEW HELPER FUNCTION to handle symbolic/numeric input
function [sym_val, num_val] = process_input(prompt)
    input_str = input(prompt, 's');
    sym_val = input_str; % Store the original string for symbolic use
    
    val = str2double(input_str);
    if isnan(val) % User entered a variable like 'd1'
        num_prompt = sprintf('   => Enter a NUMERICAL value for ''%s'' (for plotting): ', input_str);
        num_val = input(num_prompt);
    else % User entered a number
        num_val = val;
    end
end

% NEW HELPER FUNCTION to convert string to symbolic variable
function [s_var, is_symbolic_var] = convert_to_sym(input_str)
    val = str2double(input_str);
    if isnan(val) % It's a string like 'th1'
        s_var = sym(input_str);
        is_symbolic_var = true;
    else % It's a number-string like '90'
        s_var = sym(val);
        is_symbolic_var = false;
    end
end

function A_i = create_A_matrix_numeric(dh_row)
    theta_deg = dh_row(1);
    d         = dh_row(2);
    a         = dh_row(3);
    alpha_deg = dh_row(4);
    
    theta_rad = deg2rad(theta_deg);
    alpha_rad = deg2rad(alpha_deg);
    
    ct = cos(theta_rad); st = sin(theta_rad);
    ca = cos(alpha_rad); sa = sin(alpha_rad);
    
    A_i = [ ct   -st*ca   st*sa    a*ct;
            st    ct*ca   -ct*sa   a*st;
            0     sa       ca      d;
            0     0        0       1    ];
end

function A_i_sym = create_A_matrix_symbolic(theta, d, a, alpha)
    ct = cos(theta); st = sin(theta);
    ca = cos(alpha); sa = sin(alpha);

    A_i_sym = [ ct   -st*ca   st*sa    a*ct;
                st    ct*ca   -ct*sa   a*st;
                0     sa       ca      d;
                0     0        0       1    ];
end

function plotCoordinateFrame(T, labelText)
    origin = T(1:3,4);
    x_axis = T(1:3,1); y_axis = T(1:3,2); z_axis = T(1:3,3);
    scale = 5; 
    
    plot3([origin(1) origin(1)+scale*x_axis(1)], [origin(2) origin(2)+scale*x_axis(2)], [origin(3) origin(3)+scale*x_axis(3)], 'r', 'LineWidth', 2);
    plot3([origin(1) origin(1)+scale*y_axis(1)], [origin(2) origin(2)+scale*y_axis(2)], [origin(3) origin(3)+scale*y_axis(3)], 'g', 'LineWidth', 2);
    plot3([origin(1) origin(1)+scale*z_axis(1)], [origin(2) origin(2)+scale*z_axis(2)], [origin(3) origin(3)+scale*z_axis(3)], 'b', 'LineWidth', 2);
          
    text(origin(1), origin(2), origin(3), ['  ' labelText], 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
end