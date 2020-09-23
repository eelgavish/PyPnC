import numpy as np
from scipy.linalg import block_diag
from qpsolvers import solve_qp


class WBC(object):
    """
    Whole Body Control
    ------------------
    Usage:
        update_setting --> solve
    """
    def __init__(self, act_list):
        self._n_q_dot = len(act_list)
        self._n_active = np.count_non_zero(np.array(act_list))
        self._n_passive = self._n_q_dot - self._n_active

        # Selection matrix
        self._sa = np.zeros((self._n_active, self._n_q_dot))
        self._sv = np.zeros((self._n_passive, self._n_q_dot))
        j = k = 0
        for i in range(self._n_q_dot):
            if act_list[i]:
                self._sa(j, i) = 1.
                j += 1
            else:
                self._sv(k, i) = 1.
                k += 1

        # Assume first six is floating
        self._sf = np.zeros((6, self._n_q_dot))
        self._sf[0:6, 0:6] = np.eye(6)

        self._trq_limit = None
        self._lambda_q_ddot = 0.
        self._lambda_rf = 0.
        self._w_hierarchy = 0.

    @trq_limit.setter
    def trq_limit(self, val):
        assert val.shape[0] == self._n_active
        self._trq_limit = np.copy(val)

    @lambda_q_ddot.setter
    def lambda_q_ddot(self, val):
        self._lambda_q_ddot = val

    @lambda_rf.setter
    def lambda_rf(self, val):
        self._lambda_rf = val

    @w_hierarchy.setter
    def w_hierarchy(self, val):
        self._w_hierarchy = val

    def update_setting(self, mass_matrix, mass_matrix_inv, coriolis, gravity):
        self._mass_matrix = np.copy(mass_matrix)
        self._mass_matrix_inv = np.copy(mass_matrix_inv)
        self._coriolis = np.copy(coriolis)
        self._gravity = np.copy(gravity)

    def solve(self, task_list, contact_list):
        """
        Parameters
        ----------
        task_list (list of Task):
            Task list
        contact_list (list of Contact):
            Contact list

        Returns
        -------
        joint_trq_cmd (np.array):
            Joint trq cmd
        joint_acc_cmd (np.array):
            Joint acc cmd
        sol_rf (np.array):
            Reaction force
        """

        # ======================================================================
        # Cost
        # ======================================================================

        cost_t_mat = np.zeors((self._n_q_dot, self._n_q_dot))
        cost_t_vec = np.zeros(self._n_q_dot)

        for i, task in enumerate(task_list):
            j = task.jacobian
            j_dot_q_dot = task.jacobian_dot_q_dot
            x_ddot = task.op_cmd

            cost_t_mat += self._w_hierarchy[i] * np.dot(j.transpose(), j)
            cost_t_vec += self._w_hierarchy[i] * np.dot(
                (j_dot_q_dot - x_ddot).transpose(), j)
        # cost_t_mat += self._lambda_q_ddot * np.ones(self._n_q_dot)
        cost_t_mat += self._lambda_q_ddot * self._mass_matrix

        if contact_list is not None:
            uf_mat = np.array(
                block_diag(
                    *[contact.cone_constraint_mat
                      for contact in contact_list]))
            uf_vec = np.concatenate(
                [contact.cone_constraint_vec for contact in contact_list])
            contact_jacobian = np.concatenate(
                [contact.jacobian for contact in contact_list], axis=0)

            assert uf_mat.shape[0] == uf_vec.shape[0]
            assert uf_mat.shape[1] == contact_jacobian.shape[0]
            dim_cone_constraint, dim_contacts = uf_mat.shape

            cost_rf_mat = self._lambda_rf * np.ones(dim_contacts)
            v_f = np.zeros(dim_contacts)

            cost_mat = np.array(block_diag(
                cost_t_mat, cost_rf_mat))  # (nqdot+nc, nqdot+nc)
            cost_vec = np.concatenate([cost_t_vec, v_f])  # (nqdot+nc,)

        else:
            dim_contacts = dim_cone_constraint = 0
            cost_mat = np.copy(cost_t_mat)
            cost_vec = np.copy(cost_t_vec)

        # ======================================================================
        # Equality Constraint
        # ======================================================================

        if contact_list is not None:
            eq_mat = np.concatenate(
                (np.dot(self._sf, self._mass_matrix),
                 -np.dot(self._sf, contact_jacobian.transpose())),
                axis=1)  # (6, nqdot+nc)
        else:
            eq_mat = np.dot(self._sf, self._mass_matrix)
        eq_vec = -np.dot(self._sf, (self._coriolis + self._gravity))

        # ======================================================================
        # Inequality Constraint
        # ======================================================================

        if self._trq_limit is None:
            if contact_list is not None:
                ineq_mat = np.concatenate((np.zeros(
                    (dim_cone_constraint, self._n_qdot)), -uf_mat),
                                          axis=1)
                ineq_vec = -uf_vec
            else:
                ineq_mat = None
                ineq_vec = None

        else:
            if contact_list is not None:
                ineq_mat = np.concatenate(
                    np.concatenate((np.zeros(
                        (dim_cone_constraint, self._n_qdot)),
                                    -np.dot(self._sa, self._mass_matrix),
                                    np.dot(self._sa, self._mass_matrix)),
                                   axis=0),
                    np.concatenate(
                        (-uf_mat, np.dot(self._sa,
                                         contact_jacobian.transpose()),
                         -np.dot(self._sa, contact_jacobian.transpose())),
                        axis=0),
                    axis=1)
                ineq_vec = np.concatenate(
                    (-uf_vec,
                     np.dot(self._sa, self._coriolis + self._gravity) -
                     self._trq_limit[:, 0],
                     -np.dot(self._sa, self._coriolis + self._gravity) +
                     self._trq_limit[:, 1]))
            else:
                ineq_mat = np.concatenate(
                    (-np.dot(self._sa, self._mass_matrix),
                     np.dot(self._sa, self._mass_matrix)),
                    axis=0)
                ineq_vec = np.concatenate(
                    (np.dot(self._sa, self._coriolis + self._gravity) -
                     self._trq_limit[:, 0],
                     -np.dot(self._sa, self._coriolis + self._gravity) +
                     self._trq_limit[:, 1]))

        sol = solve_qp(cost_mat,
                       cost_vec,
                       ineq_mat,
                       ienq_vec,
                       eq_mat,
                       eq_vec,
                       solver="quadprog",
                       verbose=True)

        if contact_list is not None:
            sol_q_ddot, sol_rf = sol[:self._n_q_dot], sol[self._n_q_dot:]
        else:
            sol_q_ddot, sol_rf = sol, None

        if contact_list is not None:
            joint_trq_cmd = np.dot(
                self._sa,
                np.dot(self._mass_matrix, sol_q_ddot) + self._coriolis +
                self._gravity - np.dot(contact_jacobian.transpose(), sol_rf))
        else:
            joint_trq_cmd = np.dot(
                self._sa,
                np.dot(self._mass_matrix, sol_q_ddot) + self._coriolis +
                self._gravity)

        joint_acc_cmd = np.dot(self._sa, sol_q_ddot)

        return joint_trq_cmd, joint_acc_cmd, sol_rf
