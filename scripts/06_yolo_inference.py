"""
YOLOv8 객체 탐지

YOLOv8n (nano) 모델을 사용하여 객체 탐지를 수행한다.
합성 이미지 또는 ultralytics 샘플 이미지에서 탐지 결과를
바운딩 박스와 함께 시각화한다.

Dependencies: ultralytics, matplotlib, numpy
"""

import numpy as np
import matplotlib.pyplot as plt
import time


# ──────────────────────────────────────────────
# 합성 테스트 이미지 생성
# ──────────────────────────────────────────────

def create_test_scene(width=640, height=480):
    """
    YOLOv8 테스트를 위한 합성 이미지를 생성한다.
    실제 사물처럼 보이는 도형을 배치한다.
    """
    try:
        import cv2
        # 배경: 실내 느낌
        img = np.ones((height, width, 3), dtype=np.uint8) * 220

        # 바닥
        cv2.rectangle(img, (0, height * 2 // 3), (width, height), (180, 170, 160), -1)

        # 벽과 바닥 경계선
        cv2.line(img, (0, height * 2 // 3), (width, height * 2 // 3), (150, 150, 150), 2)

        # 창문 (사각형)
        cv2.rectangle(img, (50, 50), (200, 200), (135, 206, 235), -1)
        cv2.rectangle(img, (50, 50), (200, 200), (100, 100, 100), 3)
        cv2.line(img, (125, 50), (125, 200), (100, 100, 100), 2)
        cv2.line(img, (50, 125), (200, 125), (100, 100, 100), 2)

        # 테이블 (갈색 사각형)
        cv2.rectangle(img, (250, 300), (550, 380), (60, 90, 139), -1)
        # 테이블 다리
        cv2.rectangle(img, (260, 380), (275, height), (50, 70, 120), -1)
        cv2.rectangle(img, (535, 380), (550, height), (50, 70, 120), -1)

        # 컵 (테이블 위)
        cv2.rectangle(img, (350, 270), (390, 300), (200, 200, 220), -1)
        cv2.ellipse(img, (370, 270), (20, 8), 0, 0, 360, (180, 180, 200), -1)

        # 사람 형태 (간단한 실루엣)
        # 머리
        cv2.circle(img, (500, 120), 30, (200, 170, 150), -1)
        # 몸
        cv2.rectangle(img, (475, 150), (525, 280), (50, 50, 150), -1)
        # 다리
        cv2.rectangle(img, (475, 280), (498, 380), (40, 40, 100), -1)
        cv2.rectangle(img, (502, 280), (525, 380), (40, 40, 100), -1)

        # 책 (테이블 위)
        cv2.rectangle(img, (280, 280), (340, 300), (180, 50, 50), -1)

        # 화분
        cv2.rectangle(img, (60, 280), (110, 350), (139, 90, 43), -1)
        cv2.ellipse(img, (85, 280), (30, 40), 0, 180, 360, (34, 139, 34), -1)

        return img

    except ImportError:
        # OpenCV 없을 경우 단순 이미지
        img = np.random.randint(100, 200, (height, width, 3), dtype=np.uint8)
        return img


# ──────────────────────────────────────────────
# 메인
# ──────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("YOLOv8 객체 탐지 데모")
    print("=" * 60)

    try:
        from ultralytics import YOLO
    except ImportError:
        print("\n[오류] ultralytics 패키지가 설치되어 있지 않습니다.")
        print("설치 명령어: pip install ultralytics")
        print("\n대안으로 합성 결과를 보여줍니다.\n")

        # 폴백: 합성 결과
        img = create_test_scene()
        fig, ax = plt.subplots(1, 1, figsize=(10, 7))
        ax.imshow(img[..., ::-1] if img.shape[2] == 3 else img)
        ax.set_title("YOLOv8 데모 (ultralytics 미설치 - 합성 이미지만 표시)", fontsize=13)
        ax.axis("off")
        plt.tight_layout()
        save_path = "/Users/alex/Downloads/robotics-practice/scripts/06_yolo_inference.png"
        plt.savefig(save_path, dpi=120)
        print(f"그래프 저장 완료: {save_path}")
        plt.show()
        exit(0)

    # 1) 모델 로드
    print("\n[1단계] YOLOv8n (nano) 모델 로드 중...")
    model = YOLO("yolov8n.pt")  # 없으면 자동 다운로드
    print("  모델 로드 완료")

    # 2) 이미지 준비 — ultralytics 기본 샘플 사용 시도, 없으면 합성 이미지
    print("\n[2단계] 테스트 이미지 준비 중...")

    try:
        # ultralytics에 포함된 bus.jpg 샘플
        from ultralytics.data.utils import DATASETS_DIR
        import os
        sample_path = os.path.join(str(DATASETS_DIR), "bus.jpg")
        if not os.path.exists(sample_path):
            raise FileNotFoundError
        import cv2
        img_bgr = cv2.imread(sample_path)
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        source = sample_path
        print(f"  샘플 이미지 사용: {sample_path}")
    except Exception:
        # 합성 이미지 생성
        img_bgr = create_test_scene()
        img_rgb = img_bgr[:, :, ::-1].copy()
        source = img_bgr
        print("  합성 이미지 생성 완료")

    print(f"  이미지 크기: {img_rgb.shape}")

    # 3) 추론 실행
    print("\n[3단계] YOLOv8 추론 실행 중...")
    start_time = time.time()
    results = model(source, verbose=False)
    inference_time = time.time() - start_time

    result = results[0]

    # 4) 결과 출력
    print(f"\n{'=' * 60}")
    print("탐지 결과")
    print("=" * 60)
    print(f"  추론 시간: {inference_time * 1000:.1f} ms")

    boxes = result.boxes
    n_detections = len(boxes)
    print(f"  탐지된 객체 수: {n_detections}")

    if n_detections > 0:
        print(f"\n  {'번호':>4} | {'클래스':>12} | {'신뢰도':>8} | {'바운딩 박스 (x1,y1,x2,y2)':>30}")
        print("  " + "-" * 62)

        class_names = result.names
        for i, box in enumerate(boxes):
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            xyxy = box.xyxy[0].cpu().numpy()
            cls_name = class_names[cls_id]
            print(f"  {i+1:4d} | {cls_name:>12} | {conf:8.3f} | "
                  f"({xyxy[0]:.0f}, {xyxy[1]:.0f}, {xyxy[2]:.0f}, {xyxy[3]:.0f})")

        # 클래스별 통계
        print(f"\n  [클래스별 탐지 수]")
        class_counts = {}
        for box in boxes:
            cls_name = class_names[int(box.cls[0])]
            class_counts[cls_name] = class_counts.get(cls_name, 0) + 1
        for cls, cnt in sorted(class_counts.items(), key=lambda x: -x[1]):
            print(f"    {cls}: {cnt}개")

    # 5) 시각화
    print("\n[4단계] 결과 시각화 중...")

    # ultralytics의 plot 기능 사용
    annotated = result.plot()  # BGR numpy 배열
    annotated_rgb = annotated[:, :, ::-1]  # BGR → RGB

    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    # 원본
    axes[0].imshow(img_rgb)
    axes[0].set_title("원본 이미지", fontsize=14)
    axes[0].axis("off")

    # 탐지 결과
    axes[1].imshow(annotated_rgb)
    axes[1].set_title(f"YOLOv8 탐지 결과 ({n_detections}개 객체, "
                      f"{inference_time * 1000:.0f}ms)", fontsize=14)
    axes[1].axis("off")

    plt.tight_layout()
    save_path = "/Users/alex/Downloads/robotics-practice/scripts/06_yolo_inference.png"
    plt.savefig(save_path, dpi=120)
    print(f"\n그래프 저장 완료: {save_path}")
    plt.show()
