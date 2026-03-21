"""
SO(3) Exp/Log Map 구현

리 군(Lie Group) SO(3)의 지수/로그 맵을 Rodrigues 공식으로 구현한다.
축-각(axis-angle) 표현과 회전 행렬 사이의 변환, 비가환성 시연,
그리고 좌표 프레임 회전 애니메이션(정적 스냅샷)을 보여준다.

Dependencies: numpy, matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


# ──────────────────────────────────────────────
# 핵심 함수들
# ──────────────────────────────────────────────

def hat(omega: np.ndarray) -> np.ndarray:
    """hat 연산: 3-벡터 → 3×3 반대칭 행렬 [ω]×"""
    x, y, z = omega
    return np.array([
        [0, -z,  y],
        [z,  0, -x],
        [-y, x,  0]
    ])


def vee(Omega: np.ndarray) -> np.ndarray:
    """vee 연산: 3×3 반대칭 행렬 → 3-벡터 (hat의 역연산)"""
    return np.array([Omega[2, 1], Omega[0, 2], Omega[1, 0]])


def exp_so3(omega: np.ndarray) -> np.ndarray:
    """
    지수 맵: so(3) → SO(3)
    Rodrigues 공식: exp([ω]×) = I + sin(θ)/θ [ω]× + (1-cos(θ))/θ² [ω]×²
    θ = ‖ω‖
    """
    theta = np.linalg.norm(omega)
    if theta < 1e-10:
        # θ ≈ 0일 때 1차 테일러 근사
        return np.eye(3) + hat(omega)

    Omega = hat(omega)
    Omega2 = Omega @ Omega
    # Rodrigues 공식 적용
    R = (np.eye(3)
         + (np.sin(theta) / theta) * Omega
         + ((1.0 - np.cos(theta)) / (theta ** 2)) * Omega2)
    return R


def log_so3(R: np.ndarray) -> np.ndarray:
    """
    로그 맵: SO(3) → so(3) (3-벡터 반환)
    회전 행렬에서 축-각 벡터를 복원한다.
    """
    cos_theta = (np.trace(R) - 1.0) / 2.0
    # 수치 안정성을 위해 클리핑
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = np.arccos(cos_theta)

    if theta < 1e-10:
        # θ ≈ 0: 거의 항등 행렬
        return vee(R - np.eye(3))

    if abs(theta - np.pi) < 1e-6:
        # θ ≈ π: 특수 처리 — 대각 원소에서 축 복원
        diag = np.diag(R)
        k = np.argmax(diag)
        v = np.zeros(3)
        v[k] = 1.0
        # R = I + 2 sin(π) [...] + 2(1-cos(π))/π² [...]  →  R + I = 2aaᵀ 에 가까움
        col = (R[:, k] + np.eye(3)[:, k])
        col = col / np.linalg.norm(col)
        return col * theta

    # 일반적인 경우
    Omega = (theta / (2.0 * np.sin(theta))) * (R - R.T)
    return vee(Omega)


# ──────────────────────────────────────────────
# 3D 좌표 프레임 그리기 유틸리티
# ──────────────────────────────────────────────

def draw_frame(ax, R, origin=np.zeros(3), length=1.0, labels=("X", "Y", "Z")):
    """주어진 회전 행렬 R에 대응하는 좌표 프레임을 3D 축에 그린다."""
    colors = ["r", "g", "b"]
    for i in range(3):
        direction = R[:, i] * length
        ax.quiver(
            origin[0], origin[1], origin[2],
            direction[0], direction[1], direction[2],
            color=colors[i], arrow_length_ratio=0.1, linewidth=2
        )
        tip = origin + direction * 1.15
        ax.text(tip[0], tip[1], tip[2], labels[i], color=colors[i], fontsize=9)


# ──────────────────────────────────────────────
# 메인 데모
# ──────────────────────────────────────────────

if __name__ == "__main__":
    np.set_printoptions(precision=6, suppress=True)

    # ─── 1) 왕복 변환(round-trip) 검증 ───
    print("=" * 60)
    print("[데모 1] Exp → Log 왕복 변환 검증")
    print("=" * 60)

    # 임의의 축-각 벡터
    axis = np.array([1.0, 2.0, 3.0])
    axis = axis / np.linalg.norm(axis)
    angle = 1.2  # 라디안
    omega_original = axis * angle

    print(f"원본 축-각 벡터 ω      : {omega_original}")
    R = exp_so3(omega_original)
    print(f"exp(ω) 결과 (회전 행렬 R):\n{R}")

    # 회전 행렬 검증: R^T R = I, det(R) = 1
    print(f"R^T @ R ≈ I 확인       : {np.allclose(R.T @ R, np.eye(3))}")
    print(f"det(R) ≈ 1 확인        : {np.linalg.det(R):.6f}")

    omega_recovered = log_so3(R)
    print(f"log(R) 복원 결과       : {omega_recovered}")
    print(f"왕복 오차 ‖ω - log(exp(ω))‖: {np.linalg.norm(omega_original - omega_recovered):.2e}")
    print()

    # ─── 2) 비가환성(non-commutativity) 시연 ───
    print("=" * 60)
    print("[데모 2] 회전의 비가환성: exp(a)@exp(b) ≠ exp(a+b)")
    print("=" * 60)

    a = np.array([0.5, 0.0, 0.0])  # x축 회전
    b = np.array([0.0, 0.8, 0.0])  # y축 회전

    Ra = exp_so3(a)
    Rb = exp_so3(b)

    # 방법 1: 개별 exp 후 합성
    R_compose = Ra @ Rb

    # 방법 2: 벡터를 먼저 더한 뒤 exp
    R_sum = exp_so3(a + b)

    diff = np.linalg.norm(R_compose - R_sum)
    print(f"a = {a}")
    print(f"b = {b}")
    print(f"exp(a) @ exp(b) =\n{R_compose}")
    print(f"exp(a + b)      =\n{R_sum}")
    print(f"‖exp(a)@exp(b) - exp(a+b)‖ = {diff:.6f}  (0이 아니면 비가환)")
    print()

    # ─── 3) hat/vee 검증 ───
    print("=" * 60)
    print("[데모 3] hat / vee 연산 검증")
    print("=" * 60)

    v = np.array([1.0, -2.0, 3.0])
    print(f"v             = {v}")
    print(f"hat(v)        =\n{hat(v)}")
    print(f"vee(hat(v))   = {vee(hat(v))}")
    print(f"round-trip OK : {np.allclose(v, vee(hat(v)))}")
    print()

    # ─── 4) 시각화: θ를 0→π 로 변화시키며 좌표 프레임 회전 ───
    print("=" * 60)
    print("[데모 4] 좌표 프레임 회전 시각화 (θ: 0 → π)")
    print("=" * 60)

    rotation_axis = np.array([0.0, 0.0, 1.0])  # z축 기준 회전
    n_frames = 8  # 그릴 프레임 수
    thetas = np.linspace(0, np.pi, n_frames)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    for i, theta_val in enumerate(thetas):
        omega_i = rotation_axis * theta_val
        R_i = exp_so3(omega_i)
        # 프레임마다 약간의 투명도 차이
        alpha = 0.3 + 0.7 * (i / (n_frames - 1))
        draw_frame(ax, R_i, length=1.0,
                   labels=(f"X{i}", f"Y{i}", f"Z{i}"))

    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("SO(3) Exp Map: z축 기준 θ = 0 → π 회전")
    plt.tight_layout()
    plt.savefig("/Users/alex/Downloads/robotics-practice/scripts/01_lie_group.png", dpi=120)
    print("그래프 저장 완료: 01_lie_group.png")
    plt.show()
