import copy
import cvxpy as cp
import numpy as np
from scipy.sparse import csr_matrix


class SparseQuadPFMPC:
    """
    Predecessor follower linear MPC controller.

    Parameters:
        Q: move suppression cost term
        Q_p: predecessor relative error cost term
        R: input cost
        A: dynamics matrix
        B: input matrix
        C: output/sensing matrix
        u_min/max: input bounds (assume box constraints)
        H: horizon
    """

    def __init__(
        self,
        Q: np.ndarray,
        Q_p: np.ndarray,
        R: np.ndarray,
        A: np.ndarray,
        B: np.ndarray,
        C: np.ndarray,
        u_min: float,
        u_max: float,
        H: int,
    ):
        self.Ad, self.Bd, self.Cd = A, B, C
        n = self.Ad.shape[0]
        m = self.Bd.shape[1]
        p = self.Cd.shape[0]
        z_dim = (H - 1) * n + H * m + (3 * H - 2) * p
        self.n = n
        self.m = m
        self.p = p
        self.H = H

        self.z_dim = z_dim

        # cost function: assume Q, Q_p, R all diagonal
        Q_bar = np.zeros((z_dim, z_dim))
        Q_tilde = np.diag(
            np.concatenate(
                (np.zeros(n), np.diag(R), np.zeros(p), np.diag(Q), np.diag(Q_p))
            )
        )
        Q_bar[:m, :m] = R
        Q_bar[m + p :, m + p :] = np.kron(np.eye(H - 1), Q_tilde)
        self.Q_bar = csr_matrix(Q_bar)

        # equality constraint: Az = b
        A = np.zeros((H * n + (3 * H - 2) * p, z_dim))

        A_tilde_1 = np.zeros((n + p, m + p))
        A_tilde_1[:n, :m] = -self.Bd
        A_tilde_1[-p:, -p:] = np.eye(p)

        A_tilde_2 = np.zeros((n + 3 * p, n + m + 3 * p))
        A_tilde_2[:n, :n] = -self.Ad
        A_tilde_2[:n, n : n + m] = -self.Bd
        A_tilde_2[n : n + p, :n] = -self.Cd
        A_tilde_2[n : n + p, n + m : n + m + p] = np.eye(2)
        A_tilde_2[n + p : n + 2 * p, n + m : n + m + p] = np.eye(2)
        A_tilde_2[n + p : n + 2 * p, n + m + p : n + m + 2 * p] = -np.eye(2)
        A_tilde_2[n + 2 * p : n + 3 * p, n + m : n + m + p] = np.eye(2)
        A_tilde_2[n + 2 * p : n + 3 * p, n + m + 2 * p : n + m + 3 * p] = -np.eye(2)

        A_tilde_3 = copy.deepcopy(A_tilde_2)
        A_tilde_3[:n, : n + m] *= -1

        I_tilde_1 = np.zeros((n + p, n + m + 3 * p))
        I_tilde_1[:n, :n] = np.eye(n)

        I_tilde_2 = np.zeros((n + 3 * p, n + m + 3 * p))
        I_tilde_2[:n, :n] = np.eye(n)

        A = np.zeros((H * n + (3 * H - 2) * p, z_dim))
        A[: n + p, : m + p] += A_tilde_1
        A[: n + p, m + p : n + 2 * m + 4 * p] += I_tilde_1
        A[n + p : -(n + 3 * p), m + p : -(n + m + 3 * p)] += np.kron(
            np.eye(H - 2), A_tilde_2
        )
        A[n + p : -(n + 3 * p), n + 2 * m + 4 * p :] += np.kron(
            np.eye(H - 2), I_tilde_2
        )
        A[-(n + 3 * p) :, -(n + m + 3 * p) :] += A_tilde_3
        self.A = csr_matrix(A)

        # inequality constraint: Cz <= d
        C_tilde = np.zeros((2 * m, n + m + 3 * p))
        C_tilde[:m, n : n + m] += -np.eye(m)
        C_tilde[m : 2 * m, n : n + m] += np.eye(m)

        C = np.zeros((2 * H * m, z_dim))
        C[:m, :m] += -np.eye(m)
        C[m : 2 * m, :m] += np.eye(m)
        C[2 * m :, m + p :] += np.kron(np.eye(H - 1), C_tilde)
        self.C = csr_matrix(C)

        self.d = np.repeat(np.array([-u_min, u_max]), H)

    def control(self, x_0, y_a, y_p_a, d_des):
        prob, z = self.mpc_problem(x_0, y_a, y_p_a, d_des)
        prob.solve()

        z_opt = z.value

        x_opt = np.zeros((self.n, self.H + 1))
        u_opt = np.zeros((self.m, self.H))
        y_opt = np.zeros((self.p, self.H + 1))
        x_opt[:, 0] = x_0
        u_opt[:, 0] = z_opt[: self.m]
        y_opt[:, 0] = z_opt[self.m : self.m + self.p]
        for t, i in enumerate(
            range(self.m + self.p, len(z_opt), self.n + self.m + 3 * self.p)
        ):
            x_opt[:, t + 1] = z_opt[i : i + self.n]
            u_opt[:, t + 1] = z_opt[i + self.n : i + self.n + self.m]
            y_opt[:, t + 1] = z_opt[i + self.n + self.m : i + self.n + self.m + self.p]
        x_opt[:, self.H] = (
            self.Ad @ x_opt[:, self.H - 1] + self.Bd @ u_opt[:, self.H - 1]
        )
        y_opt[:, self.H] = self.Cd @ x_opt[:, self.H]

        return u_opt, x_opt, prob

    def mpc_problem(self, x_0, y_a, y_p_a, d_des):
        n, m, p, H = self.n, self.m, self.p, self.H
        z = cp.Variable(self.z_dim)
        cost = cp.quad_form(z, self.Q_bar)

        assert y_a.shape == (
            p,
            H + 1,
        ), f"y_a dim should be {(p, H + 1)}, was {y_a.shape}"
        assert y_p_a.shape == (
            p,
            H + 1,
        ), f"y_p_a dim should be {(p, H + 1)}, was {y_p_a.shape}"
        d_tilde = np.array([d_des, 0])

        b = np.zeros(self.A.shape[0])
        b[:n] += self.Ad @ x_0
        b[n : n + p] += self.Cd @ x_0
        for k, i in enumerate(range(n + p, len(b), n + 3 * p)):
            b[i + n + p : i + n + 2 * p] += y_a[:, k + 1]
            b[i + n + 2 * p : i + n + 3 * p] += y_p_a[:, k + 1] - d_tilde
        b[-(3 * p + n) : -(2 * p + n)] += y_p_a[:, H] - d_tilde

        constraints = [self.A @ z == b, self.C @ z <= self.d]
        prob = cp.Problem(cp.Minimize(cost), constraints)

        return prob, z
