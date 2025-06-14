function X = compute_max_pos_invariant_set(P, A_cl)
  % Compute the maximal positive invariant set for the closed-loop system
  % defined by the matrix A_cl and the polytope P.
  max_itr = 100;
  
  % Initialize the set with the polytope P
  X = P;
  for i = 1:max_itr
    H = X.A;
    h = X.b;

    H_prev = H * A_cl;
    h_prev = h;

    X_prev = Polyhedron('A', H_prev, 'b', h_prev);
    X_new = X_prev & P;

    if X_new == X
      break;
    end

    X = X_new;
  end

  if i == max_itr
    warning('Max iterations reached without convergence.');
  end
end