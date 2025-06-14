# Question 19

In order to choose a terminal set for ensuring closed-loop dynamics stability and constraint satisfaction, we use the LQR controller's J_inf input gain matrix, and use the dynamics $$x_{k+1} = (A+BF_{inf})x_k$$ to get the maximal invariant set and intersect it with the state constraints polytope. This works because ensuring that we reach the set of states that can be driven by the LQR controller to be asymptotic stability after the horizon.

# Question 28

It is hard to perform the minkowski sum of the propagated disturbance with the previous set until saturation.