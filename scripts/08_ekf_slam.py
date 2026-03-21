"""
EKF SLAM 2D (from scratch)

numpy만으로 구현한 2D Extended Kalman Filter SLAM.
로봇이 사각형 경로를 따라 이동하면서 5개의 랜드마크를 관측한다.
모션 모델(예측 단계)과 거리-방위 관측 모델(갱신 단계)을 구현하고,
실제 궤적, 추정 궤적, 랜드마크 위치, 불확실성 타원을 시각화한다.

Dependencies: numpy, matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


# ──────────────────────────────────────────────
# 유틸리티 함수
# ──────────────────────────────────────────────

def wrap_angle(angle):
    """각도를 [-π, π) 범위로 정규화한다."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def get_covariance_ellipse(mean, cov, n_std=2.0, n_points=50):
    """
    2D 가우시안의 공분산 타원 점들을 반환한다.
    n_std: 표준편차 배수 (2σ ≈ 95% 신뢰 구간)
    """
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    # 고유값이 음수일 수 있으므로 최소 0으로 클리핑
    eigenvalues = np.maximum(eigenvalues, 0)

    angle = np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0])
    width = 2 * n_std * np.sqrt(eigenvalues[0])
    height = 2 * n_std * np.sqrt(eigenvalues[1])

    theta = np.linspace(0, 2 * np.pi, n_points)
    ellipse_x = (width / 2) * np.cos(theta)
    ellipse_y = (height / 2) * np.sin(theta)

    # 회전 적용
    cos_a, sin_a = np.cos(angle), np.sin(angle)
    rotated_x = cos_a * ellipse_x - sin_a * ellipse_y + mean[0]
    rotated_y = sin_a * ellipse_x + cos_a * ellipse_y + mean[1]

    return rotated_x, rotated_y


# ──────────────────────────────────────────────
# EKF SLAM 클래스
# ──────────────────────────────────────────────

class EKFSLAM:
    """2D EKF SLAM 구현"""

    def __init__(self, n_landmarks, motion_noise, obs_noise):
        """
        초기화.
        state: [x, y, θ, lm1_x, lm1_y, lm2_x, lm2_y, ...]
        n_landmarks: 랜드마크 수
        motion_noise: [σ_v, σ_ω] 속도/각속도 노이즈
        obs_noise: [σ_r, σ_φ] 거리/방위 관측 노이즈
        """
        self.n_landmarks = n_landmarks
        self.state_dim = 3 + 2 * n_landmarks  # 로봇(3) + 랜드마크(2 × N)

        # 상태 벡터 초기화
        self.mu = np.zeros(self.state_dim)

        # 공분산 행렬 초기화
        # 로봇 포즈: 작은 불확실성 (시작점을 알고 있다고 가정)
        # 랜드마크: 매우 큰 불확실성 (아직 관측하지 않음)
        self.Sigma = np.eye(self.state_dim) * 1e6
        self.Sigma[:3, :3] = np.diag([0.01, 0.01, 0.01])

        # 노이즈 파라미터
        self.R = np.diag([motion_noise[0] ** 2, motion_noise[1] ** 2])  # 모션 노이즈
        self.Q = np.diag([obs_noise[0] ** 2, obs_noise[1] ** 2])        # 관측 노이즈

        # 랜드마크 관측 여부 추적
        self.landmark_seen = [False] * n_landmarks

    def predict(self, v, omega, dt):
        """
        예측 단계: 모션 모델을 적용한다.
        v: 선속도
        omega: 각속도
        dt: 시간 간격
        """
        theta = self.mu[2]

        # 상태 예측 (오도메트리 모션 모델)
        if abs(omega) > 1e-6:
            # 원호 운동
            dx = -v / omega * np.sin(theta) + v / omega * np.sin(theta + omega * dt)
            dy = v / omega * np.cos(theta) - v / omega * np.cos(theta + omega * dt)
        else:
            # 직진 운동
            dx = v * np.cos(theta) * dt
            dy = v * np.sin(theta) * dt

        self.mu[0] += dx
        self.mu[1] += dy
        self.mu[2] = wrap_angle(self.mu[2] + omega * dt)

        # 야코비안 Fx (모션 모델의 상태에 대한 편미분)
        Fx = np.eye(self.state_dim)
        if abs(omega) > 1e-6:
            Fx[0, 2] = -v / omega * np.cos(theta) + v / omega * np.cos(theta + omega * dt)
            Fx[1, 2] = -v / omega * np.sin(theta) + v / omega * np.sin(theta + omega * dt)
        else:
            Fx[0, 2] = -v * np.sin(theta) * dt
            Fx[1, 2] = v * np.cos(theta) * dt

        # 노이즈 야코비안 Fv (제어 입력에 대한 편미분)
        Fv = np.zeros((self.state_dim, 2))
        if abs(omega) > 1e-6:
            Fv[0, 0] = (-np.sin(theta) + np.sin(theta + omega * dt)) / omega
            Fv[0, 1] = v * (np.sin(theta) - np.sin(theta + omega * dt)) / (omega ** 2) \
                        + v * np.cos(theta + omega * dt) * dt / omega
            Fv[1, 0] = (np.cos(theta) - np.cos(theta + omega * dt)) / omega
            Fv[1, 1] = -v * (np.cos(theta) - np.cos(theta + omega * dt)) / (omega ** 2) \
                        + v * np.sin(theta + omega * dt) * dt / omega
        else:
            Fv[0, 0] = np.cos(theta) * dt
            Fv[1, 0] = np.sin(theta) * dt
        Fv[2, 1] = dt

        # 공분산 예측
        self.Sigma = Fx @ self.Sigma @ Fx.T + Fv @ self.R @ Fv.T

    def update(self, landmark_id, z_range, z_bearing):
        """
        갱신 단계: 랜드마크 관측을 사용하여 상태를 보정한다.
        landmark_id: 관측된 랜드마크 번호
        z_range: 관측된 거리
        z_bearing: 관측된 방위각
        """
        j = landmark_id
        lm_idx = 3 + 2 * j  # 상태 벡터에서 랜드마크 인덱스

        # 첫 관측이면 랜드마크 위치 초기화
        if not self.landmark_seen[j]:
            self.mu[lm_idx] = self.mu[0] + z_range * np.cos(self.mu[2] + z_bearing)
            self.mu[lm_idx + 1] = self.mu[1] + z_range * np.sin(self.mu[2] + z_bearing)
            self.landmark_seen[j] = True

        # 예측 관측값 계산
        dx = self.mu[lm_idx] - self.mu[0]
        dy = self.mu[lm_idx + 1] - self.mu[1]
        q = dx ** 2 + dy ** 2
        sqrt_q = np.sqrt(q)

        z_pred = np.array([
            sqrt_q,
            wrap_angle(np.arctan2(dy, dx) - self.mu[2])
        ])

        # 관측 야코비안 H
        H = np.zeros((2, self.state_dim))

        # 로봇 포즈에 대한 편미분
        H[0, 0] = -dx / sqrt_q
        H[0, 1] = -dy / sqrt_q
        H[0, 2] = 0
        H[1, 0] = dy / q
        H[1, 1] = -dx / q
        H[1, 2] = -1

        # 랜드마크 위치에 대한 편미분
        H[0, lm_idx] = dx / sqrt_q
        H[0, lm_idx + 1] = dy / sqrt_q
        H[1, lm_idx] = -dy / q
        H[1, lm_idx + 1] = dx / q

        # 혁신(innovation)
        z_actual = np.array([z_range, z_bearing])
        innovation = z_actual - z_pred
        innovation[1] = wrap_angle(innovation[1])

        # 칼만 이득
        S = H @ self.Sigma @ H.T + self.Q
        K = self.Sigma @ H.T @ np.linalg.inv(S)

        # 상태 및 공분산 갱신
        self.mu = self.mu + K @ innovation
        self.mu[2] = wrap_angle(self.mu[2])
        I_KH = np.eye(self.state_dim) - K @ H
        self.Sigma = I_KH @ self.Sigma


# ──────────────────────────────────────────────
# 시뮬레이션 환경
# ──────────────────────────────────────────────

def simulate():
    """EKF SLAM 시뮬레이션을 실행한다."""

    # 시뮬레이션 파라미터
    dt = 0.1                # 시간 간격 (초)
    n_steps = 120           # 시뮬레이션 스텝 수
    v_cmd = 1.0             # 명령 선속도 (m/s)
    max_obs_range = 5.0     # 최대 관측 거리 (m)

    # 랜드마크 위치 (고정)
    landmarks_gt = np.array([
        [2.0,  2.0],
        [5.0,  1.0],
        [4.0,  4.0],
        [1.0,  5.0],
        [3.0, -1.0],
    ])
    n_landmarks = len(landmarks_gt)

    # 노이즈 파라미터
    motion_noise_std = [0.1, 0.05]   # [σ_v, σ_ω]
    obs_noise_std = [0.15, 0.05]     # [σ_r, σ_φ]

    # EKF SLAM 초기화
    ekf = EKFSLAM(n_landmarks, motion_noise_std, obs_noise_std)

    # 기록용 리스트
    true_trajectory = []
    est_trajectory = []
    est_covariances = []  # 로봇 포즈 공분산 기록

    # 로봇 실제 상태
    true_state = np.array([0.0, 0.0, 0.0])  # [x, y, θ]

    # 사각형 경로 제어: (선속도, 각속도, 지속 스텝)
    # 직진 → 좌회전 → 직진 → 좌회전 → ... 반복
    control_sequence = []
    steps_straight = 25
    steps_turn = 5
    omega_turn = np.pi / (2 * steps_turn * dt)  # 90도 회전

    for _ in range(4):  # 4변
        control_sequence.extend([(v_cmd, 0.0)] * steps_straight)
        control_sequence.extend([(0.0, omega_turn)] * steps_turn)

    # 시뮬레이션 루프
    print("[시뮬레이션 시작]")
    for step in range(min(n_steps, len(control_sequence))):
        v, omega = control_sequence[step]

        # ── 실제 로봇 이동 (노이즈 포함) ──
        v_noisy = v + np.random.randn() * motion_noise_std[0]
        omega_noisy = omega + np.random.randn() * motion_noise_std[1]

        # 실제 상태 업데이트
        if abs(omega_noisy) > 1e-6:
            true_state[0] += -v_noisy / omega_noisy * np.sin(true_state[2]) \
                             + v_noisy / omega_noisy * np.sin(true_state[2] + omega_noisy * dt)
            true_state[1] += v_noisy / omega_noisy * np.cos(true_state[2]) \
                             - v_noisy / omega_noisy * np.cos(true_state[2] + omega_noisy * dt)
        else:
            true_state[0] += v_noisy * np.cos(true_state[2]) * dt
            true_state[1] += v_noisy * np.sin(true_state[2]) * dt
        true_state[2] = wrap_angle(true_state[2] + omega_noisy * dt)

        true_trajectory.append(true_state.copy())

        # ── EKF 예측 단계 ──
        ekf.predict(v, omega, dt)

        # ── 관측 및 EKF 갱신 단계 ──
        for lm_id, lm_pos in enumerate(landmarks_gt):
            dx = lm_pos[0] - true_state[0]
            dy = lm_pos[1] - true_state[1]
            true_range = np.sqrt(dx ** 2 + dy ** 2)

            # 관측 범위 내에 있는 랜드마크만 관측
            if true_range < max_obs_range:
                true_bearing = wrap_angle(np.arctan2(dy, dx) - true_state[2])

                # 관측에 노이즈 추가
                z_range = true_range + np.random.randn() * obs_noise_std[0]
                z_bearing = true_bearing + np.random.randn() * obs_noise_std[1]

                # EKF 갱신
                ekf.update(lm_id, z_range, z_bearing)

        est_trajectory.append(ekf.mu[:3].copy())
        est_covariances.append(ekf.Sigma[:2, :2].copy())

    true_trajectory = np.array(true_trajectory)
    est_trajectory = np.array(est_trajectory)

    return (true_trajectory, est_trajectory, est_covariances,
            landmarks_gt, ekf, n_landmarks)


# ──────────────────────────────────────────────
# 시각화
# ──────────────────────────────────────────────

def plot_results(true_traj, est_traj, est_covs, landmarks_gt, ekf, n_landmarks):
    """EKF SLAM 결과를 시각화한다."""
    fig, ax = plt.subplots(1, 1, figsize=(12, 10))

    # 실제 궤적
    ax.plot(true_traj[:, 0], true_traj[:, 1],
            "b-", linewidth=2, label="실제 궤적", alpha=0.8)
    ax.plot(true_traj[0, 0], true_traj[0, 1],
            "bs", markersize=12, label="시작점")

    # 추정 궤적
    ax.plot(est_traj[:, 0], est_traj[:, 1],
            "r--", linewidth=2, label="EKF 추정 궤적", alpha=0.8)

    # 로봇 위치 불확실성 타원 (매 10 스텝마다)
    for i in range(0, len(est_covs), 10):
        ex, ey = get_covariance_ellipse(est_traj[i, :2], est_covs[i], n_std=2.0)
        ax.plot(ex, ey, "r-", linewidth=0.5, alpha=0.4)

    # 실제 랜드마크 위치
    ax.scatter(landmarks_gt[:, 0], landmarks_gt[:, 1],
               c="green", s=200, marker="*", zorder=5,
               edgecolors="black", linewidths=1, label="실제 랜드마크")

    # 추정된 랜드마크 위치 + 불확실성 타원
    for j in range(n_landmarks):
        lm_idx = 3 + 2 * j
        if ekf.landmark_seen[j]:
            lm_est = ekf.mu[lm_idx:lm_idx + 2]
            lm_cov = ekf.Sigma[lm_idx:lm_idx + 2, lm_idx:lm_idx + 2]

            ax.scatter(lm_est[0], lm_est[1],
                       c="orange", s=120, marker="D", zorder=5,
                       edgecolors="black", linewidths=1)

            # 불확실성 타원
            ex, ey = get_covariance_ellipse(lm_est, lm_cov, n_std=2.0)
            ax.plot(ex, ey, "orange", linewidth=1.5, alpha=0.7)

            # 랜드마크 번호
            ax.annotate(f"L{j}", (lm_est[0], lm_est[1]),
                        textcoords="offset points", xytext=(10, 10),
                        fontsize=10, color="darkorange", fontweight="bold")

    # 추정 랜드마크 범례용 더미
    ax.scatter([], [], c="orange", s=120, marker="D",
               edgecolors="black", linewidths=1, label="추정 랜드마크")

    ax.set_xlabel("X (m)", fontsize=12)
    ax.set_ylabel("Y (m)", fontsize=12)
    ax.set_title("2D EKF SLAM: 로봇 궤적 및 랜드마크 추정", fontsize=14)
    ax.legend(fontsize=11, loc="upper left")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    save_path = "/Users/alex/Downloads/robotics-practice/scripts/08_ekf_slam.png"
    plt.savefig(save_path, dpi=120)
    print(f"\n그래프 저장 완료: {save_path}")
    plt.show()


# ──────────────────────────────────────────────
# 메인
# ──────────────────────────────────────────────

if __name__ == "__main__":
    np.random.seed(42)
    np.set_printoptions(precision=4, suppress=True)

    print("=" * 60)
    print("2D EKF SLAM (from scratch)")
    print("=" * 60)
    print()

    # 시뮬레이션 실행
    (true_traj, est_traj, est_covs,
     landmarks_gt, ekf, n_landmarks) = simulate()

    # 결과 출력
    print(f"\n{'=' * 60}")
    print("EKF SLAM 결과")
    print("=" * 60)

    # 궤적 오차
    pos_errors = np.linalg.norm(true_traj[:, :2] - est_traj[:, :2], axis=1)
    print(f"\n[궤적 위치 오차]")
    print(f"  평균 오차: {np.mean(pos_errors):.4f} m")
    print(f"  최대 오차: {np.max(pos_errors):.4f} m")
    print(f"  최종 오차: {pos_errors[-1]:.4f} m")

    # 랜드마크 추정 결과
    print(f"\n[랜드마크 추정 결과]")
    print(f"  {'ID':>3} | {'실제 위치':>16} | {'추정 위치':>16} | {'오차 (m)':>10}")
    print("  " + "-" * 52)

    for j in range(n_landmarks):
        lm_idx = 3 + 2 * j
        if ekf.landmark_seen[j]:
            lm_gt = landmarks_gt[j]
            lm_est = ekf.mu[lm_idx:lm_idx + 2]
            err = np.linalg.norm(lm_gt - lm_est)
            print(f"  L{j:1d} | ({lm_gt[0]:6.2f}, {lm_gt[1]:6.2f}) | "
                  f"({lm_est[0]:6.2f}, {lm_est[1]:6.2f}) | {err:10.4f}")
        else:
            print(f"  L{j:1d} | 미관측")

    # 최종 로봇 포즈
    print(f"\n[최종 로봇 포즈]")
    print(f"  실제: x={true_traj[-1, 0]:.3f}, y={true_traj[-1, 1]:.3f}, "
          f"θ={np.degrees(true_traj[-1, 2]):.1f}°")
    print(f"  추정: x={est_traj[-1, 0]:.3f}, y={est_traj[-1, 1]:.3f}, "
          f"θ={np.degrees(est_traj[-1, 2]):.1f}°")

    # 시각화
    plot_results(true_traj, est_traj, est_covs, landmarks_gt, ekf, n_landmarks)
