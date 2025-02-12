function D = create_numdiff_matrix(T_a, n, N, variant, T_a_MPC, MPC_traj_indices, param_golay)
arguments
    T_a (1,1) double
    n (1,1) double
    N (1,1) double
    variant char {mustBeMember(variant, {'fwdbwdcentral', 'bwd', 'savgol', 'savgol_notequidist', 'fwdbwdcentraltwotimes', 'fwdbwdcentralthreetimes'})} = 'fwdbwdcentral'
    T_a_MPC = T_a
    MPC_traj_indices = 1:N
    param_golay struct = struct('Nq', 2, 'd', 2)
end

    import casadi.*;

    % if(T_a == T_a_MPC && (strcmp(variant, 'fwdbwdcentraltwotimes') || strcmp(variant, 'fwdbwdcentralthreetimes')))
    %     variant = 'fwdbwdcentral';
    % end

    if(strcmp(variant, 'fwdbwdcentral'))
        % Create the matrix S_v
        E = SX.eye(n);
        S_v = SX.zeros(n * N, n * N);
        S_v(1:n, 1:2*n) = [-2*E 2*E];

        for i = 1:N-2
            S_v(1+n*i:n*(i+1), 1+n*(i-1):n*(i+2)) = [-E SX.zeros(n, n) E];
        end
        S_v(1+end-n:end, 1+end-2*n:end) = [-2*E 2*E];

        S_v = S_v / (2 * T_a);
        D = S_v;
    elseif(strcmp(variant, 'bwd'))
        % Create the matrix S_v
        E = SX.eye(n);
        S_v = SX.zeros(n * N, n * N);

        for i = 0:N-2
            S_v(1+n*i:n*(i+1), 1+n*i:n*(i+2)) = [-E E];
        end
        S_v(1+end-n:end, 1+end-2*n:end) = [-E E];

        S_v = S_v / T_a;
        D = S_v;
    elseif(strcmp(variant, 'savgol'))
        Nq = param_golay.Nq;
        d = param_golay.d;
        DD = create_savgol_deviation_matrices(T_a, Nq, d, N);
        % idea: S_v is first created for one scalar state and then for a n
        % dimensional data it is necessary to use this data multiple time
        DD_vec = cell(1, d);
        for l=1:1:d+1
            S0_v = DD{l};
            S_v = SX.zeros(n*N, n*N);
            for i=1:1:N
                for j=1:1:N
                    S_v((i-1)*n + [1:n], (j-1)*n + [1:n]) = SX.eye(n) * S0_v(i, j);
                end
            end
            DD_vec{l} = S_v;
        end
        D = DD_vec;
    elseif(strcmp(variant, 'savgol_notequidist'))
        % Warum auch immer klappt es für Nq=1 überhaupt nicht???? Dann gibt die MPC nur 0 aus
        Nq = param_golay.Nq;
        d = param_golay.d;
        DD = create_savgol_deviation_matrices_not_equidist(T_a, Nq, d, N, MPC_traj_indices);
        % idea: S_v is first created for one scalar state and then for a n
        % dimensional data it is necessary to use this data multiple time
        DD_vec = cell(1, d);
        for l=1:1:d+1
            S0_v = DD{l};
            S_v = SX.zeros(n*N, n*N);
            for i=1:1:N
                for j=1:1:N
                    S_v((i-1)*n + [1:n], (j-1)*n + [1:n]) = SX.eye(n) * S0_v(i, j);
                end
            end
            DD_vec{l} = S_v;
        end
        D = DD_vec;
    elseif(strcmp(variant, 'fwdbwdcentraltwotimes'))
        % Ta    Ta_MPC-Ta        Ta_MPC                  Ta_MPC
        % <--><------------><---------------->  ... <--------------->
        % |---|-------------|-----------------| ... |----------------|--------> t
        % 0  Ta           Ta_MPC         2Ta_MPC  (N-2)Ta_MPC    (N-1)Ta_MPC        
        % q0  q1            q2                q3   qN-1              qN

        % Ta ... Abtastzeit des CT Reglers und Sensoren
        % Ta_MPC ... Interne Abtastzeit der MPC: So grob wird Trajektorie abgetastet

        if(N<3)
            error('N must be at least 3');
        else
            E = SX.eye(n);
            S_v = SX.zeros(n * N, n * N);
            S_v(1:n, 1:2*n) = [-E E]/T_a;

            for i = 1:N-2
                if(i == 1)
                    S_v(1+n*i:n*(i+1), 1+n*(i-1):n*(i+2)) = [-E SX.zeros(n, n) E]/T_a_MPC;
                    dt_last = T_a_MPC-T_a;
                elseif(i==2)
                    S_v(1+n*i:n*(i+1), 1+n*(i-1):n*(i+2)) = [-E SX.zeros(n, n) E]/(2*T_a_MPC-T_a);
                    dt_last = T_a_MPC;
                else
                    S_v(1+n*i:n*(i+1), 1+n*(i-1):n*(i+2)) = [-E SX.zeros(n, n) E] / (2*T_a_MPC);
                end
            end
            S_v(1+end-n:end, 1+end-2*n:end) = [-E E]/dt_last;
            D = if_else(T_a == T_a_MPC, create_numdiff_matrix(T_a, n, N, 'fwdbwdcentral', T_a_MPC), S_v);
        end
    elseif(strcmp(variant, 'fwdbwdcentralthreetimes'))
        % Ta  Ta  Ta_MPC-2Ta     Ta_MPC                Ta_MPC
        % <--><--><--------><---------------->  ... <--------------->
        % |---|---|---------|-----------------| ... |----------------|--------> t
        % 0  Ta  2Ta      Ta_MPC         2Ta_MPC  (N-3)Ta_MPC    (N-2)Ta_MPC        
        % q0  q1  q2        q3                q4   qN-1              qN
        
        % Ta ... Abtastzeit des CT Reglers und Sensoren
        % Ta_MPC ... Interne Abtastzeit der MPC: So grob wird Trajektorie abgetastet

        if(N<3)
            error('N must be at least 3');
        else
            E = SX.eye(n);
            S_v = SX.zeros(n * N, n * N);
            S_v(1:n, 1:2*n) = [-E E]/T_a;

            for i = 1:N-2
                if(i == 1)
                    S_v(1+n*i:n*(i+1), 1+n*(i-1):n*(i+2)) = [-E SX.zeros(n, n) E]/(2*T_a);
                    dt_last = T_a;
                elseif(i==2)
                    S_v(1+n*i:n*(i+1), 1+n*(i-1):n*(i+2)) = [-E SX.zeros(n, n) E]/(T_a_MPC-T_a);
                    dt_last = T_a_MPC-2*T_a;
                elseif(i==3)
                    S_v(1+n*i:n*(i+1), 1+n*(i-1):n*(i+2)) = [-E SX.zeros(n, n) E] / (2*T_a_MPC - 2*T_a);
                    dt_last = T_a_MPC;
                else
                    S_v(1+n*i:n*(i+1), 1+n*(i-1):n*(i+2)) = [-E SX.zeros(n, n) E] / (2*T_a_MPC);
                end
            end
            S_v(1+end-n:end, 1+end-2*n:end) = [-E E]/dt_last;
            D = if_else(T_a == T_a_MPC, create_numdiff_matrix(T_a, n, N, 'fwdbwdcentral', T_a_MPC), S_v);
        end
    else
        error(['Unknown variant: ' variant, ': choose from {fwdbwdcentral, bwd, savgol, fwdbwdcentraltwotimes, fwdbwdcentralthreetimes}']);
    end
end