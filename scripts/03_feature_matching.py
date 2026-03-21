"""
ORB 특징점 검출 및 매칭

합성 이미지를 생성하고, 알려진 호모그래피를 적용하여 변환 이미지를 만든 뒤,
ORB 특징점을 검출하고 BFMatcher + Lowe의 비율 테스트로 매칭한다.
매칭 결과를 시각화하여 저장한다.

Dependencies: numpy, opencv-python, matplotlib
"""

import numpy as np
import cv2
import matplotlib.pyplot as plt


# ──────────────────────────────────────────────
# 합성 이미지 생성
# ──────────────────────────────────────────────

def create_synthetic_image(width=480, height=360):
    """
    다양한 도형과 텍스트가 포함된 합성 이미지를 생성한다.
    특징점이 풍부하도록 텍스처와 패턴을 추가한다.
    """
    img = np.ones((height, width, 3), dtype=np.uint8) * 240

    # 배경에 격자 무늬 추가 (특징점 검출에 도움)
    for x in range(0, width, 30):
        cv2.line(img, (x, 0), (x, height), (200, 200, 200), 1)
    for y in range(0, height, 30):
        cv2.line(img, (0, y), (width, y), (200, 200, 200), 1)

    # 사각형들
    cv2.rectangle(img, (50, 50), (150, 130), (0, 0, 200), 3)
    cv2.rectangle(img, (200, 60), (320, 160), (0, 150, 0), -1)
    cv2.rectangle(img, (30, 200), (120, 300), (180, 100, 50), 2)

    # 원들
    cv2.circle(img, (380, 100), 50, (200, 0, 0), 3)
    cv2.circle(img, (300, 280), 40, (0, 0, 180), -1)
    cv2.circle(img, (150, 250), 30, (100, 180, 0), 2)

    # 삼각형
    pts = np.array([[250, 220], [200, 320], [300, 320]], np.int32)
    cv2.polylines(img, [pts], True, (0, 100, 200), 3)

    # 텍스트 (특징점이 풍부한 영역 생성)
    cv2.putText(img, "ROBOTICS", (60, 340),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (50, 50, 50), 2)
    cv2.putText(img, "ORB", (350, 300),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 3)

    # 작은 점들 (코너 특징용)
    for _ in range(40):
        cx = np.random.randint(20, width - 20)
        cy = np.random.randint(20, height - 20)
        sz = np.random.randint(3, 8)
        color = tuple(int(c) for c in np.random.randint(0, 200, 3))
        cv2.rectangle(img, (cx, cy), (cx + sz, cy + sz), color, -1)

    return img


def apply_homography(img, H):
    """알려진 호모그래피 H를 적용하여 변환된 이미지를 생성한다."""
    h, w = img.shape[:2]
    warped = cv2.warpPerspective(img, H, (w, h),
                                 borderMode=cv2.BORDER_REPLICATE)
    return warped


# ──────────────────────────────────────────────
# ORB 특징점 검출 및 매칭
# ──────────────────────────────────────────────

def detect_and_match(img1, img2, ratio_threshold=0.75):
    """
    ORB 특징점 검출 후 BFMatcher + Lowe의 비율 테스트로 매칭.
    반환: keypoints1, keypoints2, good_matches
    """
    # ORB 디텍터 생성
    orb = cv2.ORB_create(nfeatures=1000)

    # 특징점 & 디스크립터 추출
    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)

    if des1 is None or des2 is None:
        print("[경고] 디스크립터를 추출할 수 없습니다.")
        return kp1, kp2, []

    # BFMatcher (Hamming 거리, ORB는 바이너리 디스크립터이므로)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)

    # kNN 매칭 (k=2, 비율 테스트용)
    raw_matches = bf.knnMatch(des1, des2, k=2)

    # Lowe의 비율 테스트 적용
    good_matches = []
    for match_pair in raw_matches:
        if len(match_pair) == 2:
            m, n = match_pair
            if m.distance < ratio_threshold * n.distance:
                good_matches.append(m)

    # 거리순 정렬
    good_matches.sort(key=lambda x: x.distance)

    return kp1, kp2, good_matches


# ──────────────────────────────────────────────
# 결과 시각화
# ──────────────────────────────────────────────

def visualize_matches(img1, kp1, img2, kp2, good_matches, max_draw=50):
    """매칭 결과를 시각화하여 반환한다."""
    # 상위 N개 매칭만 표시
    draw_matches = good_matches[:max_draw]

    # 매칭 이미지 생성
    match_img = cv2.drawMatches(
        img1, kp1, img2, kp2, draw_matches, None,
        matchColor=(0, 200, 0),
        singlePointColor=(0, 0, 255),
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
    )
    return match_img


# ──────────────────────────────────────────────
# 메인
# ──────────────────────────────────────────────

if __name__ == "__main__":
    np.random.seed(42)

    print("=" * 60)
    print("ORB 특징점 검출 및 매칭 데모")
    print("=" * 60)

    # 1) 합성 이미지 생성
    print("\n[1단계] 합성 이미지 생성 중...")
    img1 = create_synthetic_image()

    # 2) 알려진 호모그래피 정의 (약간의 회전 + 이동 + 원근 변환)
    # 작은 변환이어야 겹치는 영역이 충분함
    H_gt = np.array([
        [0.95, -0.08,  20],
        [0.06,  0.97, -15],
        [0.0001, 0.0002, 1.0]
    ], dtype=np.float64)

    print("[2단계] 호모그래피를 적용하여 변환 이미지 생성 중...")
    img2 = apply_homography(img1, H_gt)

    # 3) 그레이스케일 변환
    gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    # 4) 특징점 검출 및 매칭
    print("[3단계] ORB 특징점 검출 및 매칭 중...")
    kp1, kp2, good_matches = detect_and_match(gray1, gray2, ratio_threshold=0.75)

    # 5) 결과 출력
    print(f"\n[결과]")
    print(f"  이미지 1 키포인트 수: {len(kp1)}")
    print(f"  이미지 2 키포인트 수: {len(kp2)}")
    print(f"  좋은 매칭 수 (비율 테스트 통과): {len(good_matches)}")
    if len(kp1) > 0:
        ratio = len(good_matches) / len(kp1) * 100
        print(f"  매칭 비율: {ratio:.1f}%")

    if len(good_matches) > 0:
        distances = [m.distance for m in good_matches]
        print(f"  매칭 거리 - 최소: {min(distances):.1f}, "
              f"최대: {max(distances):.1f}, "
              f"평균: {np.mean(distances):.1f}")

    # 6) 시각화
    print("\n[4단계] 매칭 결과 시각화 중...")
    match_img = visualize_matches(img1, kp1, img2, kp2, good_matches)

    # matplotlib으로 표시 (BGR → RGB 변환)
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))

    # 상단: 원본과 변환 이미지
    side_by_side = np.hstack([
        cv2.cvtColor(img1, cv2.COLOR_BGR2RGB),
        cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)
    ])
    axes[0].imshow(side_by_side)
    axes[0].set_title("원본 이미지 (왼쪽) vs 호모그래피 변환 이미지 (오른쪽)", fontsize=13)
    axes[0].axis("off")

    # 하단: 매칭 결과
    axes[1].imshow(cv2.cvtColor(match_img, cv2.COLOR_BGR2RGB))
    axes[1].set_title(
        f"ORB 매칭 결과: {len(good_matches)}개 좋은 매칭 "
        f"(키포인트: {len(kp1)} / {len(kp2)})", fontsize=13)
    axes[1].axis("off")

    plt.tight_layout()

    # 저장
    save_path = "/Users/alex/Downloads/robotics-practice/scripts/03_feature_matching.png"
    plt.savefig(save_path, dpi=120)
    print(f"\n그래프 저장 완료: {save_path}")

    # 매칭 이미지도 별도 저장
    match_save_path = "/Users/alex/Downloads/robotics-practice/scripts/03_match_result.png"
    cv2.imwrite(match_save_path, match_img)
    print(f"매칭 이미지 저장 완료: {match_save_path}")

    plt.show()
