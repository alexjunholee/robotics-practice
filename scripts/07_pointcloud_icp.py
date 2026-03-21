"""
Open3D 포인트 클라우드 처리 + ICP 정합

합성 포인트 클라우드를 생성하고 알려진 변환을 적용한 뒤,
ICP (Iterative Closest Point) 알고리즘으로 정합을 수행한다.
변환 행렬, 적합도(fitness), RMSE를 출력하고 결과를 시각화한다.

Dependencies: open3d, numpy
"""

import numpy as np


# ──────────────────────────────────────────────
# Open3D 임포트 시도 — 없으면 numpy 기반 간이 ICP 폴백
# ──────────────────────────────────────────────

try:
    import open3d as o3d
    USE_OPEN3D = True
    print("[정보] Open3D를 사용합니다.")
except ImportError:
    USE_OPEN3D = False
    print("[정보] Open3D가 없습니다. numpy 기반 간이 ICP를 사용합니다.")

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


# ──────────────────────────────────────────────
# 합성 포인트 클라우드 생성
# ──────────────────────────────────────────────

def create_bunny_like_cloud(n_points=2000):
    """
    Open3D 샘플 데이터가 없는 경우를 대비하여
    토끼 형태를 간단한 기하 도형(구 + 타원체 + 원기둥)으로 근사한다.
    """
    points = []

    # 몸체: 큰 타원체
    body_n = n_points // 2
    phi = np.random.uniform(0, 2 * np.pi, body_n)
    cos_theta = np.random.uniform(-1, 1, body_n)
    theta = np.arccos(cos_theta)
    r = np.random.uniform(0.8, 1.0, body_n)
    x = r * np.sin(theta) * np.cos(phi) * 1.0
    y = r * np.sin(theta) * np.sin(phi) * 0.7
    z = r * np.cos(theta) * 0.8
    points.append(np.column_stack([x, y, z]))

    # 머리: 작은 구 (위에 위치)
    head_n = n_points // 4
    phi = np.random.uniform(0, 2 * np.pi, head_n)
    cos_theta = np.random.uniform(-1, 1, head_n)
    theta = np.arccos(cos_theta)
    r = np.random.uniform(0.3, 0.5, head_n)
    x = r * np.sin(theta) * np.cos(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(theta) + 1.2
    points.append(np.column_stack([x, y, z]))

    # 귀: 두 개의 원기둥
    ear_n = n_points // 8
    for x_offset in [-0.2, 0.2]:
        t = np.random.uniform(0, 0.8, ear_n)
        angle = np.random.uniform(0, 2 * np.pi, ear_n)
        r = np.random.uniform(0.05, 0.12, ear_n)
        x = r * np.cos(angle) + x_offset
        y = r * np.sin(angle)
        z = t + 1.7
        points.append(np.column_stack([x, y, z]))

    return np.vstack(points)


def create_known_transformation():
    """알려진 변환 행렬 (회전 + 이동)을 생성한다."""
    # 회전: z축 기준 30도
    angle = np.radians(30)
    R = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle),  np.cos(angle), 0],
        [0,              0,             1]
    ])
    t = np.array([0.5, -0.3, 0.2])

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def apply_transformation(points, T):
    """포인트 클라우드에 4×4 변환 행렬을 적용한다."""
    R = T[:3, :3]
    t = T[:3, 3]
    return (R @ points.T).T + t


def add_noise(points, sigma=0.01):
    """포인트에 가우시안 노이즈를 추가한다."""
    return points + np.random.randn(*points.shape) * sigma


# ──────────────────────────────────────────────
# numpy 기반 간이 ICP (Open3D 폴백)
# ──────────────────────────────────────────────

def find_nearest_neighbors(source, target):
    """각 source 점에 대해 가장 가까운 target 점을 찾는다 (brute-force)."""
    from scipy.spatial import cKDTree
    tree = cKDTree(target)
    distances, indices = tree.query(source, k=1)
    return distances, indices


def icp_numpy(source, target, max_iterations=50, tolerance=1e-6):
    """
    numpy + scipy를 이용한 간이 ICP 구현.
    Point-to-Point ICP.
    """
    from scipy.spatial import cKDTree

    src = source.copy()
    T_total = np.eye(4)

    prev_error = float("inf")

    for iteration in range(max_iterations):
        # 1) 최근접점 탐색
        tree = cKDTree(target)
        distances, indices = tree.query(src, k=1)
        matched_target = target[indices]

        # 2) 평균 오차 계산
        mean_error = np.mean(distances)
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

        # 3) SVD로 최적 R, t 계산
        src_centroid = np.mean(src, axis=0)
        tgt_centroid = np.mean(matched_target, axis=0)

        src_centered = src - src_centroid
        tgt_centered = matched_target - tgt_centroid

        H = src_centered.T @ tgt_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        # 반사 방지
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        t = tgt_centroid - R @ src_centroid

        # 4) 변환 적용
        src = (R @ src.T).T + t

        # 누적 변환
        T_step = np.eye(4)
        T_step[:3, :3] = R
        T_step[:3, 3] = t
        T_total = T_step @ T_total

    # 최종 결과 계산
    tree = cKDTree(target)
    final_distances, _ = tree.query(src, k=1)
    rmse = np.sqrt(np.mean(final_distances ** 2))

    # fitness: 임계값(0.05) 이내인 점의 비율
    threshold = 0.05
    fitness = np.mean(final_distances < threshold)

    return T_total, fitness, rmse, src


# ──────────────────────────────────────────────
# Open3D 기반 ICP
# ──────────────────────────────────────────────

def icp_open3d(source_pts, target_pts, threshold=0.05):
    """Open3D의 ICP 정합을 수행한다."""
    source_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(source_pts)

    target_pcd = o3d.geometry.PointCloud()
    target_pcd.points = o3d.utility.Vector3dVector(target_pts)

    # 초기 변환 (항등)
    init_T = np.eye(4)

    # Point-to-Point ICP
    result = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, threshold, init_T,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200)
    )

    T_est = result.transformation
    fitness = result.fitness
    rmse = result.inlier_rmse

    # 변환 적용된 포인트
    transformed_pts = apply_transformation(source_pts, T_est)

    return T_est, fitness, rmse, transformed_pts


# ──────────────────────────────────────────────
# 시각화
# ──────────────────────────────────────────────

def plot_registration(source, target, aligned, T_gt, T_est):
    """정합 전후를 matplotlib로 시각화한다."""
    fig = plt.figure(figsize=(16, 6))

    # 서브샘플 (표시 성능)
    max_display = 800
    idx_s = np.random.choice(len(source), min(max_display, len(source)), replace=False)
    idx_t = np.random.choice(len(target), min(max_display, len(target)), replace=False)
    idx_a = np.random.choice(len(aligned), min(max_display, len(aligned)), replace=False)

    # 1) 정합 전
    ax1 = fig.add_subplot(131, projection="3d")
    ax1.scatter(source[idx_s, 0], source[idx_s, 1], source[idx_s, 2],
                c="red", s=3, alpha=0.5, label="Source (변환됨)")
    ax1.scatter(target[idx_t, 0], target[idx_t, 1], target[idx_t, 2],
                c="blue", s=3, alpha=0.5, label="Target (원본)")
    ax1.set_title("정합 전", fontsize=13)
    ax1.legend(fontsize=8, loc="upper left")

    # 2) 정합 후
    ax2 = fig.add_subplot(132, projection="3d")
    ax2.scatter(aligned[idx_a, 0], aligned[idx_a, 1], aligned[idx_a, 2],
                c="green", s=3, alpha=0.5, label="정합 결과")
    ax2.scatter(target[idx_t, 0], target[idx_t, 1], target[idx_t, 2],
                c="blue", s=3, alpha=0.5, label="Target")
    ax2.set_title("정합 후 (ICP)", fontsize=13)
    ax2.legend(fontsize=8, loc="upper left")

    # 3) 변환 행렬 비교
    ax3 = fig.add_subplot(133)
    ax3.axis("off")
    text = "Ground Truth 변환 행렬:\n"
    text += np.array2string(T_gt, precision=4, suppress_small=True)
    text += "\n\nICP 추정 변환 행렬:\n"
    text += np.array2string(T_est, precision=4, suppress_small=True)
    text += f"\n\n오차 (Frobenius norm):\n"
    text += f"  ||T_gt - T_est|| = {np.linalg.norm(T_gt - T_est):.6f}"
    ax3.text(0.05, 0.95, text, transform=ax3.transAxes,
             fontsize=10, verticalalignment="top", fontfamily="monospace",
             bbox=dict(boxstyle="round", facecolor="lightyellow", alpha=0.8))
    ax3.set_title("변환 행렬 비교", fontsize=13)

    # 공통 축 범위 설정
    for ax in [ax1, ax2]:
        all_pts = np.vstack([source[idx_s], target[idx_t]])
        margin = 0.5
        ax.set_xlim([all_pts[:, 0].min() - margin, all_pts[:, 0].max() + margin])
        ax.set_ylim([all_pts[:, 1].min() - margin, all_pts[:, 1].max() + margin])
        ax.set_zlim([all_pts[:, 2].min() - margin, all_pts[:, 2].max() + margin])
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

    plt.tight_layout()
    save_path = "/Users/alex/Downloads/robotics-practice/scripts/07_pointcloud_icp.png"
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
    print("포인트 클라우드 ICP 정합 데모")
    print("=" * 60)

    # 1) 포인트 클라우드 생성
    print("\n[1단계] 합성 포인트 클라우드 생성 중...")

    if USE_OPEN3D:
        # Open3D 샘플 데이터 시도
        try:
            mesh = o3d.data.BunnyMesh()
            bunny = o3d.io.read_triangle_mesh(mesh.path)
            target_pts = np.asarray(bunny.sample_points_uniformly(
                number_of_points=2000).points)
            print("  Stanford Bunny 데이터 사용")
        except Exception:
            target_pts = create_bunny_like_cloud(2000)
            print("  합성 형태 사용 (Bunny 데이터 로드 실패)")
    else:
        target_pts = create_bunny_like_cloud(2000)
        print("  합성 형태 사용")

    print(f"  타깃 포인트 수: {len(target_pts)}")

    # 2) 알려진 변환 적용
    T_gt = create_known_transformation()
    source_pts = apply_transformation(target_pts, T_gt)

    # 노이즈 추가
    noise_sigma = 0.01
    source_pts = add_noise(source_pts, sigma=noise_sigma)
    print(f"  소스 포인트에 노이즈 σ={noise_sigma} 추가")

    print(f"\n[2단계] Ground Truth 변환 행렬:")
    print(T_gt)

    # 3) ICP 실행
    print(f"\n[3단계] ICP 정합 실행 중...")

    # ICP는 source를 target에 맞추므로, 역변환을 추정해야 한다
    # 즉, T_est ≈ T_gt^(-1)
    if USE_OPEN3D:
        T_est, fitness, rmse, aligned_pts = icp_open3d(
            source_pts, target_pts, threshold=0.05)
    else:
        T_est, fitness, rmse, aligned_pts = icp_numpy(
            source_pts, target_pts, max_iterations=100, tolerance=1e-8)

    # 4) 결과 출력
    print(f"\n{'=' * 60}")
    print("ICP 결과")
    print("=" * 60)
    print(f"  추정 변환 행렬:")
    print(T_est)
    print(f"\n  적합도 (Fitness): {fitness:.6f}")
    print(f"  RMSE (Inlier):    {rmse:.6f}")

    # GT의 역변환과 비교
    T_gt_inv = np.linalg.inv(T_gt)
    frob_error = np.linalg.norm(T_gt_inv - T_est)
    print(f"\n  GT 역변환 행렬:")
    print(T_gt_inv)
    print(f"\n  변환 행렬 오차 (Frobenius): {frob_error:.6f}")

    # 회전 오차
    R_err = T_est[:3, :3] @ T_gt_inv[:3, :3].T
    angle_err = np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1, 1))
    print(f"  회전 오차: {np.degrees(angle_err):.4f}°")

    # 이동 오차
    t_err = np.linalg.norm(T_est[:3, 3] - T_gt_inv[:3, 3])
    print(f"  이동 오차: {t_err:.6f}")

    # 5) 시각화
    print(f"\n[4단계] 시각화 중...")
    plot_registration(source_pts, target_pts, aligned_pts, T_gt_inv, T_est)
