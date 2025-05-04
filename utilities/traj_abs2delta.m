function [delta_X, delta_U] = traj_abs2delta(X, U, x_s, u_s)
    
    nx = size(X, 1);
    nu = size(U, 1);
    N = size(U, 2);
    assert(size(X, 2) == N + 1, ...
        "The state trajectory should be 1 step longer than the input trajectory.")

    delta_X = nan(nx, N + 1);
    delta_U = nan(nu, N);
    
    for k = 1 : N
        delta_X(:, k) = X(:, k) - x_s;
        delta_U(:, k) = U(:, k) - u_s;
    end
    
    delta_X(:, end) = X(:, end) - x_s;

end