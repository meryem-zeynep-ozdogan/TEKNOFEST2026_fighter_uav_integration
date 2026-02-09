# -*- coding: utf-8 -*-
"""
Constant-turn UKF for target state estimation and short-horizon prediction.
State: [px, py, pz, vx, vy, vz, omega]
Measurement: [px, py, pz, vx, vy, vz]
"""
import math
import numpy as np


class NonlinearKalmanUKF:
    def __init__(self):
        self.n = 7
        self.m = 6
        self.x = np.zeros(self.n)
        self.P = np.diag([25.0, 25.0, 9.0, 10.0, 10.0, 4.0, 1.0])
        self.q_pos = 3.0
        self.q_vel = 4.5
        self.q_vz = 3.0
        self.q_omega = 0.8
        self.r_pos = 4.0
        self.r_vel = 3.5
        self.r_vz = 3.0
        self.alpha = 0.6
        self.beta = 2.0
        self.kappa = 0.0
        self._compute_weights()
        self.sigmas_f = None
        self.P_pred = None

    def _compute_weights(self):
        lam = self.alpha ** 2 * (self.n + self.kappa) - self.n
        self.lam = lam
        w_m = np.full(2 * self.n + 1, 0.5 / (self.n + lam))
        w_c = np.full(2 * self.n + 1, 0.5 / (self.n + lam))
        w_m[0] = lam / (self.n + lam)
        w_c[0] = lam / (self.n + lam) + (1 - self.alpha ** 2 + self.beta)
        self.Wm = w_m
        self.Wc = w_c

    def _sigma_points(self):
        S = np.linalg.cholesky((self.n + self.lam) * self.P + 1e-9 * np.eye(self.n))
        sigmas = [self.x]
        for i in range(self.n):
            sigmas.append(self.x + S[i])
            sigmas.append(self.x - S[i])
        return np.array(sigmas)

    @staticmethod
    def _process_model(x, dt):
        px, py, pz, vx, vy, vz, omega = x
        if abs(omega) > 1e-3:
            sin_wdt = math.sin(omega * dt)
            cos_wdt = math.cos(omega * dt)
            px += (vx * sin_wdt + vy * (1 - cos_wdt)) / omega
            py += (vy * sin_wdt - vx * (1 - cos_wdt)) / omega
            vx_new = vx * cos_wdt - vy * sin_wdt
            vy_new = vx * sin_wdt + vy * cos_wdt
        else:
            px += vx * dt
            py += vy * dt
            vx_new = vx
            vy_new = vy
        pz += vz * dt
        return np.array([px, py, pz, vx_new, vy_new, vz, omega])

    @staticmethod
    def _measurement_model(x):
        return x[:6]

    def predict(self, dt):
        sigmas = self._sigma_points()
        sigmas_f = np.array([self._process_model(s, dt) for s in sigmas])
        x_pred = np.sum(self.Wm[:, None] * sigmas_f, axis=0)
        P_pred = np.zeros((self.n, self.n))
        for i in range(sigmas_f.shape[0]):
            dx = (sigmas_f[i] - x_pred).reshape(-1, 1)
            P_pred += self.Wc[i] * (dx @ dx.T)
        Q = np.diag([
            self.q_pos,
            self.q_pos,
            self.q_pos,
            self.q_vel,
            self.q_vel,
            self.q_vz,
            self.q_omega,
        ])
        P_pred += Q
        self.sigmas_f = sigmas_f
        self.x = x_pred
        self.P_pred = P_pred

    def update(self, z_meas, meas_var_vel=4.0, meas_var_vz=3.0):
        Z_sigma = np.array([self._measurement_model(s) for s in self.sigmas_f])
        z_pred = np.sum(self.Wm[:, None] * Z_sigma, axis=0)
        Pz = np.zeros((self.m, self.m))
        Pxz = np.zeros((self.n, self.m))
        for i in range(Z_sigma.shape[0]):
            dz = (Z_sigma[i] - z_pred).reshape(-1, 1)
            dx = (self.sigmas_f[i] - self.x).reshape(-1, 1)
            Pz += self.Wc[i] * (dz @ dz.T)
            Pxz += self.Wc[i] * (dx @ dz.T)
        R = np.diag([
            self.r_pos,
            self.r_pos,
            self.r_pos,
            meas_var_vel,
            meas_var_vel,
            meas_var_vz,
        ])
        Pz += R
        K = Pxz @ np.linalg.inv(Pz)
        innovation = z_meas - z_pred
        self.x = self.x + (K @ innovation)
        self.P = self.P_pred - K @ Pz @ K.T

    def predict_horizon(self, t_lead):
        return self._process_model(self.x.copy(), t_lead)[:3]
