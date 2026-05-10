#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import math
from typing import Dict, List


# =========================
# 1) 聚合规则
# =========================
SUM_COLS = ["optimization_time", "traj_len_m", "traj_time_exec"]
MEAN_COLS = ["planning_time", "cost_v", "cost_a", "cost_j", "edt_mean"]
MIN_COLS = ["edt_min"]

ALL_TARGET_COLS = SUM_COLS + MEAN_COLS + MIN_COLS


def safe_float(x: str):
    """
    将字符串安全转换为 float。
    若为空、非法、NaN、inf，则返回 None。
    """
    if x is None:
        return None

    s = str(x).strip()
    if s == "":
        return None

    try:
        v = float(s)
        if math.isnan(v) or math.isinf(v):
            return None
        return v
    except Exception:
        return None


def aggregate_csv(csv_path: str) -> Dict[str, float]:
    """
    对单个 CSV 文件按规则聚合：
    - SUM_COLS: 求和
    - MEAN_COLS: 取平均
    - MIN_COLS: 取最小值

    返回:
        {
            "optimization_time": ...,
            "traj_len_m": ...,
            ...
        }
    """
    results: Dict[str, float] = {}

    # 初始化容器
    sum_acc = {col: 0.0 for col in SUM_COLS}
    mean_acc = {col: [] for col in MEAN_COLS}
    min_acc = {col: [] for col in MIN_COLS}

    with open(csv_path, "r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)

        # 检查表头
        if reader.fieldnames is None:
            raise ValueError(f"CSV 文件没有表头: {csv_path}")

        missing_cols = [c for c in ALL_TARGET_COLS if c not in reader.fieldnames]
        if missing_cols:
            raise ValueError(
                f"CSV 文件缺少必要列 {missing_cols}: {csv_path}\n"
                f"当前表头为: {reader.fieldnames}"
            )

        for row in reader:
            # 求和列
            for col in SUM_COLS:
                v = safe_float(row.get(col))
                if v is not None:
                    sum_acc[col] += v

            # 平均列
            for col in MEAN_COLS:
                v = safe_float(row.get(col))
                if v is not None:
                    mean_acc[col].append(v)

            # 最小值列
            for col in MIN_COLS:
                v = safe_float(row.get(col))
                if v is not None:
                    min_acc[col].append(v)

    # 整理结果
    for col in SUM_COLS:
        results[col] = sum_acc[col]

    for col in MEAN_COLS:
        values = mean_acc[col]
        results[col] = sum(values) / len(values) if values else ""

    for col in MIN_COLS:
        values = min_acc[col]
        results[col] = min(values) if values else ""

    return results


def append_to_total_results(total_csv_path: str, row_dict: Dict[str, object]) -> None:
    """
    将单条结果追加写入 total_results.csv。
    若文件不存在，则先写表头。
    """
    header = ["file_name"] + ALL_TARGET_COLS
    file_exists = os.path.exists(total_csv_path)

    with open(total_csv_path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=header)

        if not file_exists:
            writer.writeheader()

        writer.writerow(row_dict)


def main():
    # 当前脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # results 目录
    results_dir = os.path.join(script_dir, "results")
    if not os.path.isdir(results_dir):
        raise FileNotFoundError(f"未找到 results 目录: {results_dir}")

    # 输出文件
    total_csv_path = os.path.join(results_dir, "total_results.csv")

    # 遍历 results 中所有 csv
    csv_files: List[str] = []
    for name in os.listdir(results_dir):
        if not name.lower().endswith(".csv"):
            continue
        if name == "total_results.csv":
            continue
        csv_files.append(name)

    csv_files.sort()

    if not csv_files:
        print(f"[INFO] results 目录下没有可处理的 CSV 文件: {results_dir}")
        return

    for file_name in csv_files:
        csv_path = os.path.join(results_dir, file_name)
        try:
            agg = aggregate_csv(csv_path)

            out_row = {"file_name": file_name}
            out_row.update(agg)

            append_to_total_results(total_csv_path, out_row)
            print(f"[OK] 已处理并追加: {file_name}")

        except Exception as e:
            print(f"[ERROR] 处理失败 {file_name}: {e}")

    print(f"[DONE] 聚合完成，结果已追加到: {total_csv_path}")


if __name__ == "__main__":
    main()