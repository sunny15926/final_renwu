#!/usr/bin/env python3
"""
在真实相机采集的数字样本上（重新）训练 LeNet，带独立测试集，导出 ONNX。

用法示例：
  python3 scripts/retrain_digit.py \
      --data /home/sunny/digit_dataset \
      --out Digital-recognition-master/model/model_realtime.onnx

采集目录结构（由 traditional_detector 可视化窗口按键生成）：
  <data>/0/*.png ... <data>/5/*.png   数字 0~5
  <data>/6/*.png                      哨兵（无数字）

关键点：
- 输入预处理与线上 C++ 推理一致：白字黑底 28x28 灰度，ToTensor()（除以 255，无 mean/std）。
- 训练/测试按标签分层切分，测试集只用真实数据，报告的 test_acc 才是真实泛化精度。
- 可选 --add-original 追加原仓库素材作为训练增强（测试集仍只用真实数据）。
"""
import argparse
import glob
import os

import numpy as np
import torch
import torch.nn as nn
from PIL import Image
from torch.utils.data import DataLoader, Dataset
from torchvision import transforms

NUM_CLASSES = 7  # 0~5 + 哨兵


class LeNet(nn.Module):
    """与原仓库 learn/mymodel.py 中的 LeNet 完全一致。"""

    def __init__(self):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(1, 6, 5),
            nn.Sigmoid(),
            nn.MaxPool2d(2, 2),
            nn.Conv2d(6, 16, 5),
            nn.Sigmoid(),
            nn.MaxPool2d(2, 2),
        )
        self.fc = nn.Sequential(
            nn.Linear(16 * 4 * 4, 120),
            nn.Sigmoid(),
            nn.Linear(120, 84),
            nn.Sigmoid(),
            nn.Linear(84, NUM_CLASSES),
        )

    def forward(self, x):
        x = self.conv(x)
        return self.fc(x.view(x.size(0), -1))


class DigitDataset(Dataset):
    def __init__(self, paths, labels, transform=None):
        self.paths = paths
        self.labels = labels
        self.transform = transform

    def __len__(self):
        return len(self.paths)

    def __getitem__(self, idx):
        img = Image.open(self.paths[idx]).convert("L").resize((28, 28))
        if self.transform is not None:
            img = self.transform(img)
        return img, self.labels[idx]


def load_paths(root):
    paths, labels = [], []
    for label in range(NUM_CLASSES):
        d = os.path.join(root, str(label))
        if not os.path.isdir(d):
            continue
        for ext in ("*.png", "*.jpg", "*.jpeg"):
            for f in sorted(glob.glob(os.path.join(d, ext))):
                paths.append(f)
                labels.append(label)
    return paths, labels


def load_original(root):
    """追加原仓库素材（仅训练用）。目录结构见 make_label.py。"""
    paths, labels = [], []
    for f in glob.glob(os.path.join(root, "0-processed", "0", "*.jpg")):
        paths.append(f)
        labels.append(0)
    for num in range(1, 6):
        for f in glob.glob(os.path.join(root, "num%d-processed" % num, "num%d" % num, "*", "*.jpg")):
            paths.append(f)
            labels.append(num)
    for f in glob.glob(os.path.join(root, "sentinel-processed", "sentinel", "*", "*.jpg")):
        paths.append(f)
        labels.append(6)
    return paths, labels


def stratified_split(paths, labels, test_size, seed):
    rng = np.random.RandomState(seed)
    train_idx, test_idx = [], []
    for cls in range(NUM_CLASSES):
        idx = [i for i, l in enumerate(labels) if l == cls]
        rng.shuffle(idx)
        if len(idx) <= 1:
            train_idx += idx  # 样本太少则全进训练集，测试集无此类
        else:
            n_test = max(1, int(round(len(idx) * test_size)))
            test_idx += idx[:n_test]
            train_idx += idx[n_test:]
    return train_idx, test_idx


def evaluate(model, loader, device):
    model.eval()
    correct, total = 0, 0
    per_cls = np.zeros((NUM_CLASSES, 2), dtype=int)  # [correct, total]
    with torch.no_grad():
        for x, y in loader:
            x, y = x.to(device), y.to(device)
            pred = model(x).argmax(dim=1)
            correct += (pred == y).sum().item()
            total += y.size(0)
            for c in range(NUM_CLASSES):
                per_cls[c, 1] += (y == c).sum().item()
                per_cls[c, 0] += ((pred == c) & (y == c)).sum().item()
    model.train()
    return (correct / total if total else 0.0), per_cls


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="采集目录（含 0~6 子目录）")
    ap.add_argument("--out", default="model_realtime.onnx")
    ap.add_argument("--epochs", type=int, default=20)
    ap.add_argument("--batch-size", type=int, default=32)
    ap.add_argument("--lr", type=float, default=0.002)
    ap.add_argument("--test-size", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--add-original", default="", help="原仓库素材/num 目录，追加到训练集做增强")
    args = ap.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    paths, labels = load_paths(args.data)
    if not paths:
        raise SystemExit(f"未在 {args.data} 下找到任何样本（需要 0~6 子目录里的 png）")

    cnt = {}
    for l in labels:
        cnt[l] = cnt.get(l, 0) + 1
    print("真实数据各类样本数:", {k: cnt.get(k, 0) for k in range(NUM_CLASSES)})

    extra_paths, extra_labels = [], []
    if args.add_original:
        extra_paths, extra_labels = load_original(args.add_original)
        print(f"追加原仓库素材 {len(extra_paths)} 张（仅训练集）")

    train_idx, test_idx = stratified_split(paths, labels, args.test_size, args.seed)
    print(f"真实数据切分: train={len(train_idx)} test={len(test_idx)}")

    train_transform = transforms.Compose([
        transforms.RandomAffine(degrees=(-12, 12), translate=(0.15, 0.15), scale=(0.85, 1.15)),
        transforms.ToTensor(),
    ])
    test_transform = transforms.ToTensor()

    train_paths = [paths[i] for i in train_idx] + extra_paths
    train_labels = [labels[i] for i in train_idx] + extra_labels
    test_paths = [paths[i] for i in test_idx]
    test_labels = [labels[i] for i in test_idx]

    train_ds = DigitDataset(train_paths, train_labels, train_transform)
    test_ds = DigitDataset(test_paths, test_labels, test_transform)
    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True, num_workers=0)
    test_loader = DataLoader(test_ds, batch_size=args.batch_size, shuffle=False, num_workers=0)

    model = LeNet().to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    loss_fn = nn.CrossEntropyLoss()

    best_acc = 0.0
    for epoch in range(args.epochs):
        model.train()
        loss_sum, acc_sum, n = 0.0, 0.0, 0
        for x, y in train_loader:
            x, y = x.to(device), y.long().to(device)
            out = model(x)
            loss = loss_fn(out, y)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            loss_sum += loss.item() * y.size(0)
            acc_sum += (out.argmax(dim=1) == y).sum().item()
            n += y.size(0)
        test_acc, _ = evaluate(model, test_loader, device)
        best_acc = max(best_acc, test_acc)
        print(f"epoch {epoch:2d}  train_loss={loss_sum / max(n, 1):.3f} "
              f"train_acc={acc_sum / max(n, 1):.3f}  test_acc={test_acc:.3f}")

    print(f"\n最佳测试精度 = {best_acc:.3f}")
    _, per_cls = evaluate(model, test_loader, device)
    names = [str(i) for i in range(6)] + ["sentinel"]
    print("各类别召回(真实测试集):")
    for c in range(NUM_CLASSES):
        if per_cls[c, 1] > 0:
            print(f"  {names[c]:>8}: {per_cls[c, 0]}/{per_cls[c, 1]} = {per_cls[c, 0] / per_cls[c, 1]:.3f}")
        else:
            print(f"  {names[c]:>8}: (无样本)")

    # 导出 ONNX（与线上推理输入一致：1x1x28x28，单通道灰度）
    model = model.cpu().eval()
    dummy = torch.zeros(1, 1, 28, 28, dtype=torch.float32)
    torch.onnx.export(model, dummy, args.out, input_names=["input"], output_names=["output"],
                      opset_version=18)

    import onnx
    m = onnx.load(args.out, load_external_data=True)
    onnx.save_model(m, args.out, save_as_external_data=False)
    data_path = args.out + ".data"
    if os.path.exists(data_path):
        os.remove(data_path)
    print(f"\n已导出 ONNX: {args.out}")

    # 用 onnxruntime 校验输出与 PyTorch 一致
    import onnxruntime as ort
    sess = ort.InferenceSession(args.out, providers=["CPUExecutionProvider"])
    out_onnx = sess.run(None, {"input": dummy.numpy()})[0]
    with torch.no_grad():
        out_torch = model(dummy).numpy()
    diff = float(np.abs(out_onnx - out_torch).max())
    print(f"ONNX 校验: 输出 shape={out_onnx.shape}, 与 PyTorch 最大误差={diff:.2e}")


if __name__ == "__main__":
    main()
