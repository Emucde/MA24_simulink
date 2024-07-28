function F = integrate_casadi(f, DT, M, method)
    % Integrate the system using the specified method (RK4 or Euler)
    % Inputs:
    %   f: Function representing the system dynamics
    %   X0: Initial state
    %   U: Input vector
    %   DT: Time step
    %   M: Number of iterations
    %   method: Integration method ('RK4' or 'Euler')
    % Output:
    %   X_next: Next state after integration
    
    if length(f.sx_in) ~= 2
        error('Invalid number of inputs. The function f must have 2 inputs, i.e. f(x,u)');
    end

    X0 = casadi.SX.sym('X0', size(f.sx_in{1}));
    U  = casadi.SX.sym('U', size(f.sx_in{2}));
    X  = X0;
    
    if strcmp(method, 'RK4')
        % Runge-Kutta 4th order method
        for j = 1:M
            k1 = f(X, U);
            k2 = f(X + DT/2 * k1, U);
            k3 = f(X + DT/2 * k2, U);
            k4 = f(X + DT * k3, U);
            X = X + DT/6 * (k1 + 2*k2 + 2*k3 + k4);
        end
    elseif strcmp(method, 'SSPRK3')
        % Euler Method
        for j = 1:M
            k1 = f(X, U);
            k2 = f(X + DT * k1, U);
            k3 = f(X + DT/4 * k1 + DT/4 * k2, U);
            X = X + DT/6 * (k1 + k2 + 4*k3);
        end
    elseif strcmp(method, 'Euler')
        % Euler Method
        for j = 1:M
            X = X + f(X, U) * DT;
        end
    else
        error('Invalid integration method. Please choose either ''RK4'', ''SSPRK3'' or ''Euler''.');
    end
    
    opt = struct;
    opt.allow_free = true;
    F = casadi.Function('F', {X0, U}, {X}, opt);
end