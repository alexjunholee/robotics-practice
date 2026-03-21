"""
PnP (Perspective-n-Point) 포즈 추정

3D 점(큐브 꼭짓점)을 알려진 카메라 포즈로 2D에 투영한 뒤,
노이즈를 추가하고 cv2.solvePnP / cv2.solvePnPRansac으로
카메라 포즈를 역추정하여 ground truth와 비교한다.
3D 점과 카메라 위치를 시각화한다.

Dependencies: numpy, opencv-python, matplotlib
"""

import numpy as np
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


# ──────────────────────────────────────────────
# 3D 점 정의 (큐브 꼭짓점)
# ──────────────────────────────────────────────

def create_cube_points(size=1.0):
    """
    단위 큐브의 8개 꼭짓점 좌표를 생성한다.
    큐브 중심이 (0.5, 0.5, 0.5)에 위치.
    """
    pts = []
    for x in [0.0, size]:
        for y in [0.0, size]:
            for z in [0.0, size]:
                pts.append([x, y, z])
    return np.array(pts, dtype=np.float64)


# ──────────────────────────────────────────────
# 카메라 내부 파라미터 정의
# ──────────────────────────────────────────────

def create_camera_intrinsics():
    """가상 카메라의 내부 파라미터 (fx, fy, cx, cy)"""
    fx, fy = 800.0, 800.0
    cx, cy = 320.0, 240.0
    K = np.array([
        [fx,  0, cx],
        [ 0, fy, cy],
        [ 0,  0,  1]
    ], dtype=np.float64)
    # 왜곡 계수 없음
    dist_coeffs = np.zeros(5, dtype=np.float64)
    return K, dist_coeffs


# ──────────────────────────────────────────────
# Ground truth 포즈 정의
# ──────────────────────────────────────────────

def create_ground_truth_pose():
    """
    카메라의 ground truth 포즈를 정의한다.
    회전: x축으로 약간 기울임 + y축으로 회전
    이동: 큐브에서 약 3m 떨어진 위치
    """
    # 회전 벡터 (Rodrigues)
    rvec_gt = np.array([0.3, -0.5, 0.1], dtype=np.float64)
    # 이동 벡터
    tvec_gt = np.array([0.5, -0.3, 3.0], dtype=np.float64)

    R_gt, _ = cv2.Rodrigues(rvec_gt)
    return rvec_gt, tvec_gt, R_gt


# ──────────────────────────────────────────────
# 2D 투영 + 노이즈
# ──────────────────────────────────────────────

def project_points(object_points, rvec, tvec, K, dist_coeffs, noise_std=1.0):
    """
    3D 점을 2D로 투영하고 가우시안 노이즈를 추가한다.
    noise_std: 픽셀 단위 노이즈 표준편차
    """
    image_points, _ = cv2.projectPoints(
        object_points, rvec, tvec, K, dist_coeffs)
    image_points = image_points.reshape(-1, 2)

    # 노이즈 추가
    noise = np.random.randn(*image_points.shape) * noise_std
    noisy_points = image_points + noise

    return image_points, noisy_points


# ──────────────────────────────────────────────
# PnP 풀이 및 비교
# ──────────────────────────────────────────────

def solve_and_compare(object_points, noisy_image_points, K, dist_coeffs,
                      rvec_gt, tvec_gt, R_gt):
    """
    solvePnP와 solvePnPRansac 두 가지 방법으로 풀고 결과를 비교한다.
    """
    results = {}

    # 방법 1: solvePnP (ITERATIVE)
    success1, rvec1, tvec1 = cv2.solvePnP(
        object_points, noisy_image_points, K, dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE)
    if success1:
        R1, _ = cv2.Rodrigues(rvec1)
        results["solvePnP"] = (rvec1.flatten(), tvec1.flatten(), R1)

    # 방법 2: solvePnPRansac
    success2, rvec2, tvec2, inliers = cv2.solvePnPRansac(
        object_points, noisy_image_points, K, dist_coeffs)
    if success2:
        R2, _ = cv2.Rodrigues(rvec2)
        n_inliers = len(inliers) if inliers is not None else 0
        results["solvePnPRansac"] = (rvec2.flatten(), tvec2.flatten(), R2)
        results["ransac_inliers"] = n_inliers

    return results


def compute_errors(rvec_est, tvec_est, R_est, rvec_gt, tvec_gt, R_gt):
    """회전 오차(도)와 이동 오차(mm)를 계산한다."""
    # 회전 오차: R_err = R_est @ R_gt^T → angle
    R_err = R_est @ R_gt.T
    angle_err = np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1, 1))
    angle_err_deg = np.degrees(angle_err)

    # 이동 오차
    t_err = np.linalg.norm(tvec_est - tvec_gt)
    t_err_mm = t_err * 1000  # m → mm (가상 단위이지만 mm로 표시)

    return angle_err_deg, t_err_mm


# ──────────────────────────────────────────────
# 시각화
# ──────────────────────────────────────────────

def plot_3d_scene(object_points, R_gt, tvec_gt, results):
    """3D 점과 카메라 포즈를 시각화한다."""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")

    # 큐브 꼭짓점
    ax.scatter(object_points[:, 0], object_points[:, 1], object_points[:, 2],
               c="blue", s=80, marker="o", label="3D 점 (큐브)")

    # 큐브 엣지 그리기
    edges = [
        (0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
        (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7)
    ]
    for i, j in edges:
        ax.plot3D(*zip(object_points[i], object_points[j]),
                  color="lightblue", linewidth=0.8)

    def draw_camera(R, t, color, label, alpha=1.0):
        """카메라 위치와 방향을 그린다. 카메라 위치 = -R^T @ t"""
        cam_pos = -R.T @ t
        ax.scatter(*cam_pos, c=color, s=120, marker="^",
                   label=label, alpha=alpha)

        # 카메라 축 그리기 (작은 프레임)
        axis_len = 0.3
        axis_colors = ["r", "g", "b"]
        axis_labels = ["x", "y", "z"]
        for k in range(3):
            direction = R.T[:, k] * axis_len
            end = cam_pos + direction
            ax.plot3D([cam_pos[0], end[0]],
                      [cam_pos[1], end[1]],
                      [cam_pos[2], end[2]],
                      color=axis_colors[k], linewidth=2, alpha=alpha)

    # Ground truth 카메라
    draw_camera(R_gt, tvec_gt, "green", "GT 카메라")

    # 추정 카메라
    if "solvePnP" in results:
        _, t_est, R_est = results["solvePnP"]
        draw_camera(R_est, t_est, "red", "solvePnP 추정", alpha=0.7)

    if "solvePnPRansac" in results:
        _, t_est2, R_est2 = results["solvePnPRansac"]
        draw_camera(R_est2, t_est2, "orange", "RANSAC 추정", alpha=0.7)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("PnP 포즈 추정: 3D 장면 + 카메라 위치", fontsize=13)
    ax.legend(fontsize=10)

    plt.tight_layout()
    save_path = "/Users/alex/Downloads/robotics-practice/scripts/04_pnp_demo.png"
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
    print("PnP (Perspective-n-Point) 포즈 추정 데모")
    print("=" * 60)

    # 1) 3D 점 생성
    object_points = create_cube_points(size=1.0)
    print(f"\n[1] 3D 점 수: {len(object_points)} (큐브 꼭짓점)")

    # 2) 카메라 내부 파라미터
    K, dist_coeffs = create_camera_intrinsics()
    print(f"[2] 카메라 내부 파라미터:\n    fx={K[0,0]}, fy={K[1,1]}, cx={K[0,2]}, cy={K[1,2]}")

    # 3) Ground truth 포즈
    rvec_gt, tvec_gt, R_gt = create_ground_truth_pose()
    print(f"[3] GT 회전 벡터: {rvec_gt}")
    print(f"    GT 이동 벡터: {tvec_gt}")

    # 4) 2D 투영 + 노이즈
    noise_std = 2.0  # 픽셀
    clean_pts, noisy_pts = project_points(
        object_points, rvec_gt, tvec_gt, K, dist_coeffs, noise_std)
    print(f"[4] 노이즈 σ = {noise_std} 픽셀 추가됨")
    print(f"    투영된 2D 점 (노이즈 포함):\n{noisy_pts}")

    # 5) PnP 풀기
    print(f"\n{'=' * 60}")
    print("PnP 풀이 결과")
    print("=" * 60)

    results = solve_and_compare(
        object_points, noisy_pts, K, dist_coeffs,
        rvec_gt, tvec_gt, R_gt)

    for method_name in ["solvePnP", "solvePnPRansac"]:
        if method_name in results:
            rvec_est, tvec_est, R_est = results[method_name]
            rot_err, trans_err = compute_errors(
                rvec_est, tvec_est, R_est, rvec_gt, tvec_gt, R_gt)

            print(f"\n[{method_name}]")
            print(f"  추정 회전 벡터: {rvec_est}")
            print(f"  추정 이동 벡터: {tvec_est}")
            print(f"  회전 오차: {rot_err:.4f}°")
            print(f"  이동 오차: {trans_err:.2f} mm")
            if method_name == "solvePnPRansac" and "ransac_inliers" in results:
                print(f"  RANSAC 인라이어 수: {results['ransac_inliers']}/{len(object_points)}")

    # 6) 시각화
    plot_3d_scene(object_points, R_gt, tvec_gt, results)
