"""
Factor Graph 기반 2D Pose Graph SLAM

5개의 포즈가 사각형 경로를 따라 이동하는 간단한 2D SLAM 문제를 구현한다.
연속 포즈 사이의 오도메트리 팩터와 루프 클로저 팩터를 사용하여
그래프를 구성하고, Levenberg-Marquardt 최적화로 포즈를 보정한다.

GTSAM이 설치되어 있으면 사용하고, 없으면 scipy.optimize를 이용한 폴백을 제공한다.

Dependencies: numpy, matplotlib, (선택) gtsam 또는 scipy
"""

import numpy as np
import matplotlib.pyplot as plt

# ──────────────────────────────────────────────
# GTSAM 사용 가능 여부 확인
# ──────────────────────────────────────────────
try:
    import gtsam
    USE_GTSAM = True
    print("[정보] GTSAM을 사용합니다.")
except ImportError:
    USE_GTSAM = False
    print("[정보] GTSAM이 없습니다. scipy.optimize 폴백을 사용합니다.")
    from scipy.optimize import least_squares


# ──────────────────────────────────────────────
# 유틸리티 함수
# ──────────────────────────────────────────────

def wrap_angle(theta):
    """각도를 [-π, π) 범위로 정규화한다."""
    return (theta + np.pi) % (2 * np.pi) - np.pi


def pose2_compose(p1, p2):
    """2D 포즈 합성: p1 ⊕ p2 (x, y, θ)"""
    x1, y1, t1 = p1
    x2, y2, t2 = p2
    c, s = np.cos(t1), np.sin(t1)
    return np.array([
        x1 + c * x2 - s * y2,
        y1 + s * x2 + c * y2,
        wrap_angle(t1 + t2)
    ])


def pose2_inverse(p):
    """2D 포즈의 역변환"""
    x, y, t = p
    c, s = np.cos(t), np.sin(t)
    return np.array([
        -c * x - s * y,
        s * x - c * y,
        -t
    ])


def pose2_between(p1, p2):
    """두 포즈 사이의 상대 변환: p1⁻¹ ⊕ p2"""
    return pose2_compose(pose2_inverse(p1), p2)


# ──────────────────────────────────────────────
# 사각형 경로의 ground truth 포즈 정의
# ──────────────────────────────────────────────

def create_ground_truth_poses():
    """5개 포즈로 이루어진 사각형 경로 (시계 방향)"""
    side = 2.0
    poses = np.array([
        [0.0,    0.0,    0.0],           # 포즈 0: 출발점
        [side,   0.0,    np.pi / 2],     # 포즈 1: 오른쪽으로 이동 후 좌회전
        [side,   side,   np.pi],         # 포즈 2: 위로 이동 후 좌회전
        [0.0,    side,   -np.pi / 2],    # 포즈 3: 왼쪽으로 이동 후 좌회전
        [0.0,    0.0,    0.0],           # 포즈 4: 출발점으로 복귀 (루프 클로저)
    ])
    return poses


def create_odometry_measurements(gt_poses, noise_std):
    """
    ground truth에서 연속 포즈 사이의 오도메트리를 계산하고 노이즈를 추가한다.
    noise_std: [σ_x, σ_y, σ_θ]
    """
    measurements = []
    for i in range(len(gt_poses) - 1):
        delta = pose2_between(gt_poses[i], gt_poses[i + 1])
        # 가우시안 노이즈 추가
        noise = np.random.randn(3) * noise_std
        noisy_delta = delta + noise
        noisy_delta[2] = wrap_angle(noisy_delta[2])
        measurements.append(noisy_delta)
    return measurements


def compute_initial_poses(odom_measurements):
    """오도메트리만으로 초기 포즈를 누적 (open-loop)"""
    poses = [np.array([0.0, 0.0, 0.0])]  # 첫 포즈는 원점 고정
    for delta in odom_measurements:
        new_pose = pose2_compose(poses[-1], delta)
        poses.append(new_pose)
    return np.array(poses)


# ──────────────────────────────────────────────
# scipy.optimize 기반 폴백 최적화
# ──────────────────────────────────────────────

def optimize_with_scipy(initial_poses, odom_measurements, loop_measurement,
                        odom_noise_std, loop_noise_std):
    """
    scipy least_squares를 이용한 Pose Graph 최적화.
    첫 번째 포즈는 고정(prior)한다.
    """
    n_poses = len(initial_poses)

    # 상태 벡터 초기화 (포즈 0 제외, 고정)
    x0 = initial_poses[1:].flatten()

    # 정보 행렬의 역(가중치)
    odom_weight = 1.0 / odom_noise_std
    loop_weight = 1.0 / loop_noise_std

    def residuals(x):
        """잔차 함수: 모든 팩터에 대한 잔차 벡터"""
        # 현재 포즈 복원
        poses = np.zeros((n_poses, 3))
        poses[0] = initial_poses[0]  # 고정
        poses[1:] = x.reshape(-1, 3)

        res_list = []

        # 오도메트리 팩터 잔차
        for i, delta_meas in enumerate(odom_measurements):
            delta_est = pose2_between(poses[i], poses[i + 1])
            err = delta_est - delta_meas
            err[2] = wrap_angle(err[2])
            res_list.extend(err * odom_weight)

        # 루프 클로저 팩터 잔차 (포즈 0 ↔ 포즈 4)
        delta_loop_est = pose2_between(poses[0], poses[4])
        err_loop = delta_loop_est - loop_measurement
        err_loop[2] = wrap_angle(err_loop[2])
        res_list.extend(err_loop * loop_weight)

        return np.array(res_list)

    result = least_squares(residuals, x0, method="lm")

    optimized_poses = np.zeros((n_poses, 3))
    optimized_poses[0] = initial_poses[0]
    optimized_poses[1:] = result.x.reshape(-1, 3)
    return optimized_poses, result.cost


# ──────────────────────────────────────────────
# GTSAM 기반 최적화
# ──────────────────────────────────────────────

def optimize_with_gtsam(initial_poses, odom_measurements, loop_measurement,
                        odom_noise_std, loop_noise_std):
    """GTSAM을 이용한 Pose Graph 최적화"""
    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()

    # 노이즈 모델 생성
    odom_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
        np.array(odom_noise_std))
    loop_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
        np.array(loop_noise_std))
    prior_noise_model = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([0.01, 0.01, 0.001]))

    # 초기 포즈에 prior 팩터 추가
    graph.add(gtsam.PriorFactorPose2(
        0, gtsam.Pose2(*initial_poses[0]), prior_noise_model))

    # 초기 추정값 설정
    for i, pose in enumerate(initial_poses):
        initial_estimate.insert(i, gtsam.Pose2(*pose))

    # 오도메트리 팩터 추가
    for i, delta in enumerate(odom_measurements):
        graph.add(gtsam.BetweenFactorPose2(
            i, i + 1, gtsam.Pose2(*delta), odom_noise_model))

    # 루프 클로저 팩터 추가 (포즈 0 ↔ 포즈 4)
    graph.add(gtsam.BetweenFactorPose2(
        0, 4, gtsam.Pose2(*loop_measurement), loop_noise_model))

    # Levenberg-Marquardt 최적화
    params = gtsam.LevenbergMarquardtParams()
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate, params)
    result = optimizer.optimize()

    # 결과 추출
    optimized_poses = np.array([
        [result.atPose2(i).x(), result.atPose2(i).y(), result.atPose2(i).theta()]
        for i in range(len(initial_poses))
    ])
    final_error = graph.error(result)
    return optimized_poses, final_error


# ──────────────────────────────────────────────
# 시각화
# ──────────────────────────────────────────────

def plot_results(gt_poses, initial_poses, optimized_poses):
    """최적화 전후 비교 플롯"""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    for ax, title, poses, color, label in [
        (axes[0], "최적화 전 (Open-Loop, 노이즈 누적)",
         initial_poses, "red", "초기 추정"),
        (axes[1], "최적화 후 (루프 클로저 적용)",
         optimized_poses, "blue", "최적화 결과"),
    ]:
        # ground truth
        gt_closed = np.vstack([gt_poses, gt_poses[0]])
        ax.plot(gt_closed[:, 0], gt_closed[:, 1],
                "k--o", label="Ground Truth", markersize=8, linewidth=1.5)

        # 추정 포즈
        p_closed = np.vstack([poses, poses[0]])
        ax.plot(p_closed[:, 0], p_closed[:, 1],
                f"{color[0]}-s", label=label, markersize=8, linewidth=1.5)

        # 포즈 번호 표시
        for i, p in enumerate(poses):
            ax.annotate(f"P{i}", (p[0], p[1]),
                        textcoords="offset points", xytext=(8, 8), fontsize=10)

        # 헤딩 방향 화살표
        arrow_len = 0.25
        for p in poses:
            dx = arrow_len * np.cos(p[2])
            dy = arrow_len * np.sin(p[2])
            ax.arrow(p[0], p[1], dx, dy,
                     head_width=0.08, head_length=0.04, fc=color, ec=color)

        ax.set_title(title, fontsize=12)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.legend(fontsize=10)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(
        "/Users/alex/Downloads/robotics-practice/scripts/02_factor_graph_slam.png",
        dpi=120)
    print("그래프 저장 완료: 02_factor_graph_slam.png")
    plt.show()


# ──────────────────────────────────────────────
# 메인
# ──────────────────────────────────────────────

if __name__ == "__main__":
    np.random.seed(42)
    np.set_printoptions(precision=4, suppress=True)

    # ground truth 포즈 생성
    gt_poses = create_ground_truth_poses()

    # 오도메트리 노이즈 표준편차 (x, y, θ)
    odom_noise_std = np.array([0.1, 0.1, 0.05])
    loop_noise_std = np.array([0.05, 0.05, 0.02])

    # 노이즈가 추가된 오도메트리 측정값 생성
    odom_measurements = create_odometry_measurements(gt_poses, odom_noise_std)

    # 루프 클로저 측정값 (포즈 0 → 포즈 4, 사실상 항등 변환에 노이즈)
    loop_gt = pose2_between(gt_poses[0], gt_poses[4])
    loop_noise = np.random.randn(3) * loop_noise_std
    loop_measurement = loop_gt + loop_noise
    loop_measurement[2] = wrap_angle(loop_measurement[2])

    # 초기 추정 (오도메트리만 사용, open-loop)
    initial_poses = compute_initial_poses(odom_measurements)

    print("=" * 60)
    print("2D Pose Graph SLAM")
    print("=" * 60)
    print(f"\n[Ground Truth 포즈]")
    for i, p in enumerate(gt_poses):
        print(f"  P{i}: x={p[0]:6.3f}, y={p[1]:6.3f}, θ={np.degrees(p[2]):7.2f}°")

    print(f"\n[초기 추정 (open-loop, 노이즈 누적)]")
    for i, p in enumerate(initial_poses):
        print(f"  P{i}: x={p[0]:6.3f}, y={p[1]:6.3f}, θ={np.degrees(p[2]):7.2f}°")

    # 최적화 수행
    if USE_GTSAM:
        optimized_poses, final_error = optimize_with_gtsam(
            initial_poses, odom_measurements, loop_measurement,
            odom_noise_std, loop_noise_std)
    else:
        optimized_poses, final_error = optimize_with_scipy(
            initial_poses, odom_measurements, loop_measurement,
            odom_noise_std, loop_noise_std)

    print(f"\n[최적화 후 포즈]")
    for i, p in enumerate(optimized_poses):
        print(f"  P{i}: x={p[0]:6.3f}, y={p[1]:6.3f}, θ={np.degrees(p[2]):7.2f}°")

    # 오차 비교
    print(f"\n[오차 비교]")
    print(f"{'포즈':>6} | {'초기 오차 (m)':>14} | {'최적화 후 오차 (m)':>18}")
    print("-" * 46)
    for i in range(len(gt_poses)):
        err_init = np.linalg.norm(initial_poses[i, :2] - gt_poses[i, :2])
        err_opt = np.linalg.norm(optimized_poses[i, :2] - gt_poses[i, :2])
        print(f"  P{i}   | {err_init:14.4f} | {err_opt:18.4f}")

    print(f"\n최종 비용(cost): {final_error:.6f}")

    # 시각화
    plot_results(gt_poses, initial_poses, optimized_poses)
