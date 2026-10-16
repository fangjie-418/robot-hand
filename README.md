# 灵巧手 PID 控制仿真（Robot Hand）

一个纯前端、交互式的**机器人灵巧手（5 指 · 15 关节）PID 控制算法仿真** Web 应用。
使用 **Three.js** 构建三维灵巧手模型，**Chart.js** 绘制实时曲线，直观展示关节位置与速度的精确闭环控制。

> 零构建、零后端依赖 —— 双击 `index.html` 即可在浏览器中运行（需联网加载 CDN 资源）。

## 功能特性

### 1. 三维灵巧手模型（Three.js r128）
- 5 根手指（拇指 / 食指 / 中指 / 无名指 / 小指），每指 3 个关节（掌指 MCP / 近指 PIP / 远指 DIP），共 **15 个独立关节**。
- 所有关节可独立控制旋转角度，模型支持鼠标**旋转 / 缩放 / 平移**（触摸屏手势同样支持）。
- 关节球体随 PID 输出力矩实时变色（热力图），选中关节/手指高亮。

### 2. PID 控制实现
- 每个关节独立 PID 控制器：`u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt`。
- 关节动力学模型：`J·θ″ + B·θ′ + K·θ = τ`，τ 为 PID 输出力矩。
- **前馈重力补偿**：`τ_ff = m·g·L·cos(θ_abs)`，抵消重力负载，减轻积分项负担（可开关）。
- 积分抗饱和（anti-windup）、关节限位（±90°）、力矩限幅。

### 3. 控制面板
- 全局 PID 参数 `Kp / Ki / Kd` 滑块（0–10，精度 0.01）。
- 手指选择器 + 关节选择器（支持「全部」）。
- 目标角度设定（-90° ~ +90°）。
- 目标轨迹预设：**抓握 / 伸展 / 捏取 / 摇手（动态）/ 自定义**。

### 4. 可视化
- 关节实时角度（数字 + 进度条，含目标标记）。
- 关节角速度实时曲线。
- PID 输出力矩热力图（15 格 + 3D 模型同步着色）。
- 误差收敛曲线 e-t。
- 阶跃响应分析（超调量、调节时间 ±2%、稳态误差）。

### 5. 交互功能
- **手势录制与回放**：录制手动调整过程并回放。
- **自动调参**：Web Worker 中随机搜索 + 坐标下降，自动优化 PID 增益。
- **抗干扰测试**：注入随机扰动转矩 + 脉冲，观察 PID 恢复能力。
- **对比模式**：并行仿真两组 PID 增益，曲线叠加对比。

### 6. 仿真参数
- 仿真频率 100Hz（RK4 积分），渲染 60fps。
- 质量 / 阻尼 / 刚度倍率可调，力矩上限可调。

## 快速开始

```bash
# 方式一：直接双击打开
index.html

# 方式二：本地静态服务器（推荐，避免个别浏览器对 file:// 的限制）
python -m http.server 8000     # 然后访问 http://localhost:8000
# 或
npx serve .
```

## 技术栈

| 依赖 | 版本 | 用途 |
|------|------|------|
| Three.js | 0.128.0 (UMD) | 三维灵巧手渲染 |
| OrbitControls | 0.128.0 | 视角旋转/缩放/平移 |
| Chart.js | 4.4.1 | 实时曲线 |
| Web Worker (Blob) | - | 自动调参密集计算 |

## 项目结构

```
robot-hand/
├── index.html      # 完整应用（HTML + CSS + JS 单文件）
└── README.md
```

## 算法说明

**PID 离散化**（100Hz，`dt = 0.01s`）：

```
e(t)  = θ_target − θ(t)
∫e    += e(t) · dt            （积分，带抗饱和限幅）
de/dt = −θ′(t)                 （目标恒定时，误差导数 = −角速度）
u(t)  = Kp·e + Ki·∫e + Kd·(de/dt)
```

**动力学积分**（四阶 Runge-Kutta）：

```
dθ/dt  = ω
dω/dt  = (τ_net − B·ω − K·θ) / J
τ_net  = u + τ_ff − τ_gravity + τ_dist
```

## 发布到 GitHub（GitHub Pages）

> 若本机已安装 git 与 GitHub CLI，可执行以下命令：

```bash
cd robot-hand
git init
git add index.html README.md
git commit -m "feat: 灵巧手 PID 控制仿真 (Three.js + Chart.js)"

# 创建远程仓库（仓库名：robot-hand）
gh repo create robot-hand --public --source=. --push

# 开启 GitHub Pages
gh api -X POST repos/{owner}/robot-hand/pages \
  -f "source[branch]=main" -f "source[path]=/"
```

发布后访问：`https://<你的用户名>.github.io/robot-hand/`

> 未安装 `gh` 时，可在 GitHub 网页端手动创建空仓库 `robot-hand`，然后：
>
> ```bash
> git remote add origin https://github.com/<你的用户名>/robot-hand.git
> git branch -M main
> git push -u origin main
> ```
>
> 再到仓库 **Settings → Pages** 将 Source 设为 `main` 分支根目录即可。

## 许可证

MIT
