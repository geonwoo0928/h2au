"""
학습된 보행자 탐지 모델로 웹캠/영상을 실시간 추론하는 스크립트.

RPi4 배포를 염두에 두고 기본 경로(TorchScript .pt 로드)는 torch, cv2, numpy만
있으면 동작하도록 만듦 (pandas/albumentations 등 학습용 의존성 불필요).
raw state_dict(.pth)를 직접 로드하는 경우에만 train_person_detector.py를
불러와 모델 구조를 재구성함 (이 경로는 desktop에서의 디버깅용).

decode_prediction / nms / calculate_iou는 train_person_detector.py의 것과
반드시 동일한 수식을 유지해야 함 (학습 시 build_target의 log(bw/W) 인코딩과
decode의 exp(pred)*W가 서로의 정확한 역함수여야 박스 크기가 맞게 나옴).
"""
import argparse
import math
import time

import cv2
import numpy as np
import torch

IMAGE_WIDTH_DEFAULT = 320
IMAGE_HEIGHT_DEFAULT = 240
WH_LOG_CLAMP = 4.0

# train_person_detector.py의 PersonDataset과 반드시 동일해야 함:
# mobilenet 백본은 ImageNet 사전학습 분포에 맞춰 정규화한 입력으로 학습됨.
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def decode_prediction(pred, img_w, img_h, conf_threshold=0.25):
    boxes = []
    pred = pred.detach().cpu()
    grid_h, grid_w = pred.shape[1], pred.shape[2]
    cell_w = img_w / grid_w
    cell_h = img_h / grid_h

    obj = torch.sigmoid(pred[0])
    ys, xs = torch.where(obj > conf_threshold)

    for y, x in zip(ys.tolist(), xs.tolist()):
        # 다중 양성 셀 할당으로 오프셋 범위가 [-0.5,1.5)까지 확장됨 (train_person_detector.py의 build_target 참고)
        tx = (torch.sigmoid(pred[1, y, x]) * 2.0 - 0.5).item()
        ty = (torch.sigmoid(pred[2, y, x]) * 2.0 - 0.5).item()
        tw = torch.clamp(pred[3, y, x], max=WH_LOG_CLAMP).item()
        th = torch.clamp(pred[4, y, x], max=WH_LOG_CLAMP).item()

        bw = math.exp(tw) * img_w
        bh = math.exp(th) * img_h

        cx = (x + tx) * cell_w
        cy = (y + ty) * cell_h

        xmin = max(0.0, cx - bw / 2)
        ymin = max(0.0, cy - bh / 2)
        xmax = min(float(img_w), cx + bw / 2)
        ymax = min(float(img_h), cy + bh / 2)

        boxes.append([xmin, ymin, xmax, ymax, obj[y, x].item()])

    return boxes


def calculate_iou(box1, box2):
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])

    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
    area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
    union = area1 + area2 - inter

    return inter / union if union > 0 else 0.0


def nms(boxes, iou_threshold=0.3):
    if len(boxes) == 0:
        return []

    boxes = sorted(boxes, key=lambda b: b[4], reverse=True)
    keep = []
    while boxes:
        best = boxes.pop(0)
        keep.append(best)
        boxes = [b for b in boxes if calculate_iou(best[:4], b[:4]) < iou_threshold]

    return keep


def preprocess(frame, img_w, img_h, device, normalize=False):
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (img_w, img_h))
    img = img.astype(np.float32) / 255.0
    if normalize:
        img = (img - IMAGENET_MEAN) / IMAGENET_STD
    img = np.transpose(img, (2, 0, 1))
    tensor = torch.from_numpy(img).unsqueeze(0).to(device)
    return tensor


def load_model(weights_path, backbone, device):
    if weights_path.endswith(".pt"):
        try:
            model = torch.jit.load(weights_path, map_location=device)
            model.eval()
            print(f"TorchScript 모델 로드: {weights_path}")
            return model
        except RuntimeError:
            pass  # TorchScript가 아니면 아래에서 state_dict으로 재시도

    # raw state_dict 경로: 모델 구조가 필요하므로 학습 스크립트에서 import
    from train_person_detector import PersonDetector

    model = PersonDetector(backbone=backbone, pretrained=False).to(device)
    # weights_only=False: 자체 학습 파이프라인이 만든 체크포인트(신뢰 가능한 로컬 파일)를 로드
    ckpt = torch.load(weights_path, map_location=device, weights_only=False)
    state_dict = ckpt["model_state_dict"] if isinstance(ckpt, dict) and "model_state_dict" in ckpt else ckpt
    model.load_state_dict(state_dict)
    model.eval()
    print(f"state_dict 모델 로드: {weights_path} (backbone={backbone})")
    return model


def run(args):
    device = torch.device(args.device if args.device else ("cuda" if torch.cuda.is_available() else "cpu"))
    model = load_model(args.weights, args.backbone, device)

    source = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise RuntimeError(f"영상 소스를 열 수 없습니다: {args.source}")

    writer = None
    if args.save:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        fps_in = cap.get(cv2.CAP_PROP_FPS) or 20.0
        w0 = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h0 = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        writer = cv2.VideoWriter(args.save, fourcc, fps_in, (w0, h0))

    prev_time = time.time()
    frame_idx = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        h, w = frame.shape[:2]
        tensor = preprocess(frame, args.width, args.height, device, normalize=(args.backbone == "mobilenet"))

        with torch.no_grad():
            output = model(tensor)

        boxes = decode_prediction(output[0], args.width, args.height, args.conf)
        boxes = nms(boxes, args.nms_iou)

        sx, sy = w / args.width, h / args.height
        for xmin, ymin, xmax, ymax, conf in boxes:
            rx0 = max(0, min(w, int(xmin * sx)))
            ry0 = max(0, min(h, int(ymin * sy)))
            rx1 = max(0, min(w, int(xmax * sx)))
            ry1 = max(0, min(h, int(ymax * sy)))

            cv2.rectangle(frame, (rx0, ry0), (rx1, ry1), (0, 255, 0), 2)
            cv2.putText(frame, f"person {conf:.2f}", (rx0, max(0, ry0 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        now = time.time()
        fps = 1.0 / max(now - prev_time, 1e-6)
        prev_time = now
        cv2.putText(frame, f"FPS: {fps:.1f}  persons: {len(boxes)}", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        if writer is not None:
            writer.write(frame)
        if not args.no_display:
            cv2.imshow("Person Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        elif args.print_every and frame_idx % args.print_every == 0:
            print(f"frame {frame_idx}: FPS={fps:.1f} persons={len(boxes)}")

        frame_idx += 1

    cap.release()
    if writer is not None:
        writer.release()
    if not args.no_display:
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="보행자 탐지 실시간 추론")
    parser.add_argument("--source", default="0", help="웹캠 인덱스(예: 0) 또는 영상/이미지 파일 경로")
    parser.add_argument("--weights", default="./person_detector_script.pt")
    parser.add_argument("--backbone", choices=["custom", "mobilenet"], default="custom",
                         help="학습 때 쓴 backbone과 반드시 일치시켜야 함 "
                              "(mobilenet이면 ImageNet 정규화가 전처리에 적용됨; "
                              "raw state_dict(.pth) 로드 시엔 모델 구조 재구성에도 사용됨)")
    parser.add_argument("--width", type=int, default=IMAGE_WIDTH_DEFAULT)
    parser.add_argument("--height", type=int, default=IMAGE_HEIGHT_DEFAULT)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--nms-iou", type=float, default=0.3)
    parser.add_argument("--device", default=None)
    parser.add_argument("--save", default=None, help="결과 영상을 저장할 경로 (mp4)")
    parser.add_argument("--no-display", action="store_true", help="GUI 없이 실행 (RPi 헤드리스 환경용)")
    parser.add_argument("--print-every", type=int, default=30, help="--no-display일 때 콘솔 로그 출력 간격(프레임)")
    args = parser.parse_args()

    run(args)


if __name__ == "__main__":
    main()
