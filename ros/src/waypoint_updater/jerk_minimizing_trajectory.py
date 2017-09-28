import numpy as np

def jmt(start, end, T):
    """
    Calculates Jerk Minimizing Trajectory for start, end and T.
    """
    a_0, a_1, a_2 = start[0], start[1], start[2] / 2.0
    c_0 = a_0 + a_1 * T + a_2 * T ** 2
    c_1 = a_1 + 2 * a_2 * T
    c_2 = 2 * a_2

    A = np.array([
        [T ** 3, T ** 4, T ** 5],
        [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
        [6 * T, 12 * T ** 2, 20 * T ** 3],
    ])
    B = np.array([
        end[0] - c_0,
        end[1] - c_1,
        end[2] - c_2
    ])
    a_3_4_5 = np.linalg.solve(A, B)
    alphas = np.concatenate([np.array([a_0, a_1, a_2]), a_3_4_5])
    return alphas