import numpy as np
import matplotlib.pyplot as plt

# ===================== 中文字体（可选） =====================
# 如果你不需要中文，可以删掉这两行
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False


# ===================== 参数设置（你可以改这里） =====================

v_max = 1.0      # 👉 最大速度（整体曲线高度，越大曲线整体越高）
alpha = 2.0      # 👉 控制下降速度（越大下降越快）

theta_max = 1.5  # 👉 横轴最大角度范围（控制图的宽度）

# ============================================================


# ===================== 生成数据 =====================
theta = np.linspace(0, theta_max, 200)   # 横轴：转角 θ_k
v = v_max / (1 + alpha * theta)          # 对应公式


# ===================== 开始画图 =====================
plt.figure(figsize=(6, 4))   # 👉 图大小（论文建议 6×4 或 5×3）

plt.plot(theta, v, linewidth=2)

# ===================== 坐标轴标签 =====================
plt.xlabel(r'$\theta_k$', fontsize=12)
plt.ylabel(r'$v_k$', fontsize=12)

# ===================== 标题 =====================
# plt.title('基于转角的速度分配函数', fontsize=13)

# ===================== 网格（建议保留） =====================
plt.grid(True, linestyle='--', alpha=0.5)

# ===================== 在图中写公式 =====================
# 👉 修改 xy 控制公式位置
plt.text(
    0.55 * theta_max,   # 横坐标位置
    0.8 * v_max,        # 纵坐标位置
    r'$v_k = \frac{v_{max}}{1 + \alpha \theta_k}$',
    fontsize=14
)

# ===================== 调整边界 =====================
plt.xlim(0, theta_max)
plt.ylim(0, v_max * 1.1)

plt.tight_layout()

# ===================== 保存为矢量图 =====================
# 👉 保存路径你可以改
plt.savefig('./pics/figures/速度函数示意图.svg')
plt.savefig('./pics/figures/速度函数示意图.pdf')

plt.show()