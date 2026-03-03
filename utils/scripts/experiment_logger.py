import os
import csv
import time
from datetime import datetime


class SimpleExperimentLogger:
    """
    极简实验记录器：

    功能：
    1. 记录每一次分段规划
    2. 统计 planning / optimization 时间
    3. 内部存储为结构化数据
    4. 最终输出为 CSV
    """

    def __init__(self):
        scripts_dir = os.path.dirname(os.path.abspath(__file__))
        self.results_dir = os.path.join(scripts_dir, "results")
        os.makedirs(self.results_dir, exist_ok=True)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.results_dir, f"experiment_{ts}.csv")

        # ===== 内部状态 =====
        self.planner_name = ""
        self.plan_index = 0
        self.rows = []

        # timing
        self._t0 = None

        print(f"[LOGGER] CSV will be saved to: {self.csv_path}")

    # ==========================
    # 初始化
    # ==========================
    def start(self, planner_name: str):
        self.planner_name = planner_name
        self.plan_index = 0
        self.rows = []

    # ==========================
    # 计时工具
    # ==========================
    def tic(self):
        self._t0 = time.perf_counter()

    def toc(self):
        if self._t0 is None:
            return 0.0
        return time.perf_counter() - self._t0

    # ==========================
    # 记录一次分段
    # ==========================
    def log_segment(
            self,
            start_real,
            goal_real,
            planning_time,
            optimization_time,
            cost_v, cost_a, cost_jerk,
    ):
        self.plan_index += 1

        row = {
            "planner": self.planner_name,
            "plan_index": self.plan_index,
            "start_x": start_real[0],
            "start_y": start_real[1],
            "goal_x": goal_real[0],
            "goal_y": goal_real[1],
            "planning_time": planning_time,
            "optimization_time": optimization_time,
            "cost_v": cost_v,
            "cost_a": cost_a,
            "cost_jerk": cost_jerk,
        }

        self.rows.append(row)

    # ==========================
    # 保存 CSV
    # ==========================
    def save(self):
        if not self.rows:
            print("[LOGGER] No data to save.")
            return

        fieldnames = [
            "planner",
            "plan_index",
            "start_x",
            "start_y",
            "goal_x",
            "goal_y",
            "planning_time",
            "optimization_time",
            "cost_v",
            "cost_a",
            "cost_jerk",
        ]

        with open(self.csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.rows)

        print(f"[LOGGER] Saved {len(self.rows)} segments to {self.csv_path}")
