"""
Depth Anything V2 단안 깊이 추정

Depth Anything V2 모델을 사용하여 단일 이미지에서 깊이 맵을 추정한다.
원본 이미지와 깊이 맵을 나란히 시각화한다.

GPU 권장. CPU에서도 동작하지만 느리다.

Dependencies: transformers, torch, matplotlib, PIL, numpy
"""

import numpy as np
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings("ignore")

# ──────────────────────────────────────────────
# 합성 테스트 이미지 생성 (OpenCV 사용 가능 시)
# ──────────────────────────────────────────────

def create_sample_image(width=518, height=518):
    """
    깊이 추정 테스트용 합성 이미지를 생성한다.
    다양한 크기의 도형을 배치하여 '가까움/멀리' 느낌을 준다.
    OpenCV가 없으면 PIL로 생성한다.
    """
    try:
        import cv2
        # 하늘색 배경
        img = np.zeros((height, width, 3), dtype=np.uint8)
        # 하늘 그라데이션
        for y in range(height // 2):
            ratio = y / (height // 2)
            img[y, :] = [int(200 - 80 * ratio), int(220 - 60 * ratio), 255]
        # 바닥 (녹색 그라데이션)
        for y in range(height // 2, height):
            ratio = (y - height // 2) / (height // 2)
            img[y, :] = [int(50 + 30 * ratio), int(150 - 50 * ratio), int(50 + 20 * ratio)]

        # 먼 산 (배경)
        pts = np.array([[0, height // 2], [width // 4, height // 3],
                        [width // 2, height // 2 - 30],
                        [3 * width // 4, height // 4],
                        [width, height // 2]], np.int32)
        cv2.fillPoly(img, [pts], (100, 130, 100))

        # 중간 거리 건물
        cv2.rectangle(img, (100, 180), (180, height // 2 + 50), (80, 80, 120), -1)
        cv2.rectangle(img, (300, 200), (380, height // 2 + 50), (100, 90, 80), -1)

        # 가까운 물체 (큰 원)
        cv2.circle(img, (width // 2, height - 100), 60, (0, 0, 200), -1)
        cv2.circle(img, (width // 2, height - 100), 60, (0, 0, 150), 3)

        # 도로
        road_pts = np.array([
            [width // 2 - 100, height],
            [width // 2 + 100, height],
            [width // 2 + 20, height // 2 + 50],
            [width // 2 - 20, height // 2 + 50]
        ], np.int32)
        cv2.fillPoly(img, [road_pts], (60, 60, 60))

        # 텍스트
        cv2.putText(img, "DEPTH TEST", (width // 2 - 100, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # BGR → RGB 변환
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img

    except ImportError:
        # OpenCV 없으면 PIL로 간단한 이미지 생성
        from PIL import Image, ImageDraw
        img = Image.new("RGB", (width, height), (135, 206, 235))
        draw = ImageDraw.Draw(img)
        # 바닥
        draw.rectangle([0, height // 2, width, height], fill=(34, 139, 34))
        # 건물
        draw.rectangle([100, 180, 180, height // 2 + 50], fill=(80, 80, 120))
        draw.rectangle([300, 200, 380, height // 2 + 50], fill=(100, 90, 80))
        # 가까운 원
        draw.ellipse([width // 2 - 60, height - 160,
                      width // 2 + 60, height - 40], fill=(200, 0, 0))
        return np.array(img)


# ──────────────────────────────────────────────
# 메인
# ──────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("Depth Anything V2 단안 깊이 추정")
    print("=" * 60)
    print("GPU 권장. CPU에서도 동작하지만 느리다.\n")

    # 1) 이미지 준비
    print("[1단계] 테스트 이미지 생성 중...")
    image_np = create_sample_image()

    from PIL import Image
    image_pil = Image.fromarray(image_np)
    print(f"  이미지 크기: {image_pil.size}")

    # 2) 모델 로드
    print("\n[2단계] Depth Anything V2 모델 로드 중...")
    print("  (처음 실행 시 모델 다운로드에 시간이 걸릴 수 있습니다)")

    try:
        import torch
        from transformers import pipeline

        # 디바이스 설정
        if torch.cuda.is_available():
            device_str = "cuda"
            print("  GPU (CUDA) 사용")
        elif hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
            device_str = "mps"
            print("  Apple Silicon (MPS) 사용")
        else:
            device_str = "cpu"
            print("  CPU 사용 (느릴 수 있음)")

        # Depth Anything V2 파이프라인 생성 (small 모델)
        pipe = pipeline(
            task="depth-estimation",
            model="depth-anything/Depth-Anything-V2-Small-hf",
            device=device_str
        )

        # 3) 깊이 추정 실행
        print("\n[3단계] 깊이 추정 실행 중...")
        import time
        start_time = time.time()
        result = pipe(image_pil)
        elapsed = time.time() - start_time
        print(f"  추론 시간: {elapsed:.2f}초")

        # 결과 추출
        depth_map = np.array(result["depth"])

        print(f"\n[결과]")
        print(f"  깊이 맵 크기: {depth_map.shape}")
        print(f"  깊이 최솟값: {depth_map.min():.4f}")
        print(f"  깊이 최댓값: {depth_map.max():.4f}")
        print(f"  깊이 평균값: {depth_map.mean():.4f}")
        print(f"  깊이 표준편차: {depth_map.std():.4f}")

        model_loaded = True

    except Exception as e:
        print(f"\n[경고] 모델 로드/추론 실패: {e}")
        print("  합성 깊이 맵을 대신 생성합니다.")

        # 폴백: 간단한 합성 깊이 맵 (밝기 기반)
        gray = np.mean(image_np, axis=2)
        # 위쪽(하늘)은 멀리, 아래쪽(가까운 물체)은 가까이
        y_gradient = np.linspace(0.2, 1.0, image_np.shape[0]).reshape(-1, 1)
        depth_map = (gray / 255.0 * 0.5 + y_gradient * 0.5)
        depth_map = (depth_map * 255).astype(np.uint8)

        print(f"\n[결과 (합성 깊이)]")
        print(f"  깊이 맵 크기: {depth_map.shape}")
        print(f"  깊이 최솟값: {depth_map.min():.4f}")
        print(f"  깊이 최댓값: {depth_map.max():.4f}")

        model_loaded = False

    # 4) 시각화
    print("\n[4단계] 시각화 중...")
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # 원본 이미지
    axes[0].imshow(image_np)
    axes[0].set_title("원본 이미지", fontsize=14)
    axes[0].axis("off")

    # 깊이 맵
    im = axes[1].imshow(depth_map, cmap="inferno")
    title = "Depth Anything V2 깊이 맵" if model_loaded else "합성 깊이 맵 (모델 미사용)"
    axes[1].set_title(title, fontsize=14)
    axes[1].axis("off")
    plt.colorbar(im, ax=axes[1], fraction=0.046, pad=0.04)

    plt.tight_layout()
    save_path = "/Users/alex/Downloads/robotics-practice/scripts/05_depth_estimation.png"
    plt.savefig(save_path, dpi=120)
    print(f"그래프 저장 완료: {save_path}")
    plt.show()
