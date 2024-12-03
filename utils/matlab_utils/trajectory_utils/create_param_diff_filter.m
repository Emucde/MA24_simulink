function traj_struct_out = create_param_diff_filter(traj_struct, param_global, text1, lambda, text2, n_order, text3, n_input, type)
    arguments
        traj_struct struct {mustBeNonempty}
        param_global struct {mustBeNonempty}
        text1 char = 'lambda'
        lambda double = -10
        text2 char = 'n_order'
        n_order uint8 = 6
        text3 char = 'n_input'
        n_input uint8 = 4
        type char {mustBeMember(type, {'diff_filter', 'diff_filter_jointspace'})} = 'diff_filter'
    end

    %% Zustandsvariablenfilter
    % lambda = -10;  %Eigenwert für Sollwertfilter, die Zeitkonstante ist dann
    % ja tau = 1/(abs(lambda)) in Sekunden, d. h. bei schnelleren
    % Trajektorien muss man die Zeitkonstante des Sollwertfilters anpassen,
    % damit es den Signalen schnell genug folgen kann. Momentan auf 200 ms
    % eingestellt (lambda = -5), d. h. das Delay beträgt ca. 5 tau = 1s, d.
    % h. erst nach dieser Zeit kann man damit rechnen, dass der Ausgang des
    % Sollwertfilters mit dessen Eingang übereinstimmt.

    coeffs = poly(ones(1,n_order)*lambda);

    a0_f = coeffs(end);
    ai_f = coeffs(n_order+1:-1:2); % matlab orders it descending.
    A_f = [[zeros(n_order-1,1), eye(n_order-1)]; -ai_f];
    b_f = [zeros(n_order-1,1); a0_f];
    C_f = eye(n_order)*a0_f; % damit [yf ; d/dt yf; d^2/dt^2 yf] = a0^3 * xf gilt.
    D_f = zeros(n_order,1);

    %SYS_f = ss(A_f,b_f,C_f,D_f); % kontinuierliches System für y
    %SYS_f_z = c2d(SYS_f, param_global.Ta, 'damped'); % diskretes System für y (macht eh euler vorwärts)

    %Phi = SYS_f_z.A;
    %Gamma = SYS_f_z.B;

    %EULER:
    Phi = (eye(n_order) + A_f*param_global.Ta); % euler vorwärts
    Gamma = b_f*param_global.Ta;  % euler vorwärts

    %RK4:
    % d/dt x = Ax + bu = f(x,u)
    % k1 = f(x0,u) = Ax0 + bu
    % k2 = f(x0 + Ta/2 * k1, u) = (E+Ta/2*A)*A x0 + (A*Ta/2*b) u = A1 x0 + b1 u
    % k3 = f(x0 + Ta/2 * k2, u) = A*(E+Ta/2*A1) x0 + (A*Ta/2*b1+b) u = A2 x0 + b2 u
    % k4 = f(x0 + Ta   * k3, u) = A*(E+Ta  *A2) x0 + (A*Ta  *b2+b) u = A3 x0 + b3 u
    % x1 = x0 + Ta/6 * (k1 + 2*k2 + 2*k3 + k4) = 
    %    = (E + Ta/6 * (A + 2*A1 + 2*A2 + A3)) x0 + (b + Ta/6 * (b + 2*b1 + 2*b2 + b3)) u
    %    = Phi x0 + Gamma u
    % A = A_f;
    % b = b_f;
    % Ta = param_global.Ta;
    % E = eye(n_order);
    % A1 = A*(E+Ta/2*A);
    % A2 = A*(E+Ta/2*A1);
    % A3 = A*(E+Ta*  A2);

    % b1 = A*Ta/2*b;
    % b2 = A*Ta/2*b1+b;
    % b3 = A*Ta*b2+b;

    % Phi = (E + Ta/6 * (A + 2*A1 + 2*A2 + A3));
    % Gamma = Ta/6 * (b + 2*b1 + 2*b2 + b3);


    A_f_cells   = reshape(mat2cell(repmat(A_f, 1,1,n_input),   n_order, n_order, ones(1, n_input)), 1, []);
    b_f_cells   = reshape(mat2cell(repmat(b_f, 1,1,n_input),   n_order, 1,       ones(1, n_input)), 1, []);

    Phi_cells   = reshape(mat2cell(repmat(Phi,   1,1,n_input), n_order, n_order, ones(1, n_input)), 1, []);
    Gamma_cells = reshape(mat2cell(repmat(Gamma, 1,1,n_input), n_order, 1,       ones(1, n_input)), 1, []);

    diff_filter = struct;
    diff_filter.A = blkdiag(A_f_cells{:});
    diff_filter.b = blkdiag(b_f_cells{:});
    diff_filter.Ta = param_global.Ta;
    diff_filter.Phi = blkdiag(Phi_cells{:});
    diff_filter.Gamma = blkdiag(Gamma_cells{:});

    % Brauche ich in Simulink:
    selector_index1 = 1:n_order:n_order*n_input;
    selector_index2 = selector_index1+1;
    selector_index3 = selector_index1+2;

    diff_filter.p_d_index    = selector_index1;
    diff_filter.p_d_p_index  = selector_index2;
    diff_filter.p_d_pp_index = selector_index3;

    diff_filter.n_order = n_order;
    diff_filter.n_input = n_input;

    if strcmp(type, 'diff_filter')
        traj_struct.diff_filter = diff_filter;
    elseif strcmp(type, 'diff_filter_jointspace')
        traj_struct.diff_filter_jointspace = diff_filter;
    else
        error('type have to be diff_filter or diff_filter_joint_space');
    end

    traj_struct_out = traj_struct;

    %lamda_alpha = lambda;
    %{
    %% Filter für xyz:
    lambda = -5;  %Eigenwert für Sollwertfilter, die Zeitkonstante ist dann
    % ja tau = 1/(abs(lambda)) in Sekunden, d. h. bei schnelleren
    % Trajektorien muss man die Zeitkonstante des Sollwertfilters anpassen,
    % damit es den Signalen schnell genug folgen kann. Momentan auf 200 ms
    % eingestellt (lambda = -5), d. h. das Delay beträgt ca. 5 tau = 1s, d.
    % h. erst nach dieser Zeit kann man damit rechnen, dass der Ausgang des
    % Sollwertfilters mit dessen Eingang übereinstimmt.

    coeffs = poly([lambda lambda lambda]);

    a0_f = coeffs(4);
    a1_f = coeffs(3);
    a2_f = coeffs(2);

    A_f = [0 1 0; 0 0 1; -a0_f -a1_f -a2_f];
    b_f = [0; 0; a0_f];
    C_f = eye(3)*a0_f; % damit [yf ; d/dt yf; d^2/dt^2 yf] = a0^3 * xf gilt.
    D_f = [0; 0; 0];

    SYS_f = ss(A_f,b_f,C_f,D_f); % kontinuierliches System für y
    SYS_f_z = c2d(SYS_f, param_global.Ta); % diskretes System für y (macht eh euler
    %vorwärts)

    Phi_yt = (eye(6) + A_f*param_global.Ta); % euler vorwärts
    Gamma_yt = b_f*param_global.Ta;  % euler vorwärts

    %% Filter für alpha
    %lambda = -5;
    coeffs = poly([lambda lambda lambda]);

    a0_f = coeffs(4);
    a1_f = coeffs(3);
    a2_f = coeffs(2);

    A_f = [0 1 0; 0 0 1; -a0_f -a1_f -a2_f];
    b_f = [0; 0; a0_f];
    C_f = eye(3)*a0_f; % damit [yf ; d/dt yf; d^2/dt^2 yf] = a0^3 * xf gilt.
    D_f = [0; 0; 0];

    Phi_alpha = (eye(3) + A_f*param_global.Ta); % euler vorwärts
    Gamma_alpha = b_f*param_global.Ta;  % euler vorwärts

    traj_struct.diff_filter.A = blkdiag(A_f, A_f, A_f, A_f);
    traj_struct.diff_filter.b = blkdiag(b_f, b_f, b_f, b_f);
    traj_struct.diff_filter.Ta = param_global.Ta;
    traj_struct.diff_filter.Phi = blkdiag(Phi_yt, Phi_yt, Phi_yt, Phi_alpha);
    traj_struct.diff_filter.Gamma = blkdiag(Gamma_yt,Gamma_yt,Gamma_yt,Gamma_alpha);

    % Brauche ich in Simulink:
    selector_index1 = [1 4 7 10];
    selector_index2 = selector_index1+1;
    selector_index3 = selector_index1+2;
    %}
end