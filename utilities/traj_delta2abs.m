function [X, U] = traj_delta2abs(delta_X, delta_U, x_s, u_s)
    nx = size(delta_X, 1);
    nu = size(delta_U, 1);
    N = size(delta_U, 2);
    assert(size(delta_X, 2) == N + 1, ...
            "The state trajectory should be 1 step longer than the input trajectory.")

    X = nan(nx, N + 1);
    U = nan(nu, N);
    
    for k = 1 : N
        X(:, k) = delta_X(:, k) + x_s;
        U(:, k) = delta_U(:, k) + u_s;
    end
    
    X(:, end) = delta_X(:, end) + x_s;
end