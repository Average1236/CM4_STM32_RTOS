## 符号定义
* $r$: 轮子半径
* $l$: 轮子到机器人中心的距离
* $d$: 机器人中心到机器人质心的距离
* $\alpha$: 轮子 1 和 2 的安装角度（相对于机器人左右方向）
* $\beta$: 轮子 3 和 4 的安装角度（相对于机器人左右方向）
* $c_{\alpha} = \cos\alpha$, $s_{\alpha} = \sin\alpha$, $c_{\beta} = \cos\beta$, $s_{\beta} = \sin\beta$
* $m_1$: 机器人质量
* $J_{1E}$: 机器人转动惯量（绕垂直轴）

## 加速度规划
从远程主机接收期望速度 $\bm{v}_{1} = [v_{x,1}, v_{y,1}, \omega_1]^T$，根据目前的速度 $\bm{v}_0 = [v_{x,0} v_{y,0}, \omega_0]^T$ 和加速度 $\dot{\bm{v}}_0$，在最大加速度 $\bm{a}_{max}$ 和最大加加速度 $\bm{j}_{max}$ 的约束下，使用简单的加速度规划器生成平滑的期望加速度 $\dot{\bm{v}}_{ref}$（控制周期为 $dt$）：

将速度规划问题等价映射为“位置—速度—加速度”三层梯形轨迹规划，其中速度 $v$ 相当于位置，加速度 $a=\dot v$ 相当于速度，jerk $j=\ddot v$ 相当于加速度；在 $|a|\le a_{max}$、$|j|\le j_{max}$ 约束下，按照 $\Delta v \ge a_{max}^2/j_{max}$ 与否，分别采用“三段式 jerk 限幅 S 曲线”或“两段式 jerk 曲线”生成平滑的 $\dot{\bm v}_{ref}$。

---

### 1. 变量定义

对某一轴，设当前速度为 $v_0$，目标速度为 $v_1$，当前加速度为 $a_0=\dot v_0$。为了统一加/减速过程，定义

$$
\Delta v = v_1 - v_0,\qquad s=\operatorname{sgn}(\Delta v),\qquad \Delta = |\Delta v|.
$$

将规划问题变成“正向”问题后，有

$$
\tilde v = s,v,\qquad \tilde a = s,a,\qquad \tilde j = s,j,
$$

这样只需要讨论 ($\Delta \ge 0$) 的情况。

约束为

$$
|a|\le a_{max},\qquad |j|\le j_{max}.
$$

---

### 2. 规划判据

把速度梯形规划中的公式逐项替换后，原文“是否存在匀速段”的判据变成了这里的“是否存在恒加速度段”：

$$
\Delta \ge \frac{a_{max}^2}{j_{max}}
$$

则存在恒加速度段；否则不存在，只能走“jerk 上升—jerk 下降”的两段式。

---

### 3. 情况 A：存在恒加速度段

此时

$$
T_j = \frac{a_{max}}{j_{max}},
\qquad
T_a = \frac{\Delta}{a_{max}}-\frac{a_{max}}{j_{max}},
\qquad
T = 2T_j+T_a.
$$

加速度参考轨迹为

$$
a_{ref}(t)=
\begin{cases}
sj_{max}(t-t_0), & t_0\le t<t_0+T_j,\\
sa_{max}, & t_0+T_j\le t<t_0+T_j+T_a,\\
s\left(a_{max}-j_{max}(t-t_0-T_j-T_a)\right), & t_0+T_j+T_a\le t\le t_0+T.
\end{cases}
$$

对应的速度参考轨迹为

$$
v_{ref}(t)=
\begin{cases}
v_0+\dfrac{1}{2}sj_{max}(t-t_0)^2, & t_0\le t<t_0+T_j,\\
v_0+sa_{max}\left(t-t_0-\dfrac{T_j}{2}\right), & t_0+T_j\le t<t_0+T_j+T_a,\\
v_1-\dfrac{1}{2}sj_{max}(t_0+T-t)^2, & t_0+T_j+T_a\le t\le t_0+T.
\end{cases}
$$

---

### 4. 情况 B：不存在恒加速度段

当

$$
\Delta < \frac{a_{max}^2}{j_{max}}
$$

时，没有中间恒加速度段，只能走两段式。此时峰值加速度会低于 $a_{max}$，并且
$$
T_j=\sqrt{\frac{\Delta}{j_{max}}},\qquad T=2T_j,\qquad a_{pk}=j_{max}T_j=\sqrt{j_{max}\Delta}.
$$

加速度参考轨迹为

$$
a_{ref}(t)=
\begin{cases}
sj_{max}(t-t_0), & t_0\le t<t_0+T_j,\\
sj_{max}(t_0+T-t), & t_0+T_j\le t\le t_0+T.
\end{cases}
$$

速度参考轨迹为

$$
v_{ref}(t)=
\begin{cases}
v_0+\dfrac{1}{2}sj_{max}(t-t_0)^2, & t_0\le t<t_0+T_j,\\
v_1-\dfrac{1}{2}sj_{max}(t_0+T-t)^2, & t_0+T_j\le t\le t_0+T.
\end{cases}
$$

---

### 5. 在线离散实现

如果要在控制周期 $dt$ 下实时输出“期望加速度” $\dot{\bm v}_{ref}$，最实用的写法是对每个通道独立执行：

$$
j_{cmd}[k] =
\operatorname{sat}\left(\frac{a_{des}[k]-a[k]}{dt},-j_{max},j_{max}\right),
$$

$$
a_{ref}[k+1]=
\operatorname{sat}\left(a_{ref}[k]+j_{cmd}[k]dt,-a_{max},a_{max}\right),
$$

$$
v[k+1]=v[k]+a_{ref}[k+1]dt.
$$

其中 $a_{des}$ 是“当前希望达到的加速度方向”，最简单的取法是

$$
a_{des}[k]=
\operatorname{sat}\left(k_v\bigl(v_{ref}-v[k]\bigr),-a_{max},a_{max}\right),
$$

然后再用 jerk 限幅把它平滑跟踪掉。这样得到的 $a_{ref}$ 就是要发给下一级的 $\dot{\bm v}_{ref}$。


## 期望力矩计算

### 最小二乘法求伪逆矩阵
已知
$$
\bm{J}_1 = \frac{1}{r} \begin{bmatrix}
    c_{\alpha} & -c_{\alpha} & -c_{\beta} & c_{\beta} \\
    -s_{\alpha} & -s_{\alpha} & s_{\beta} & s_{\beta} \\
    -l - ds_\alpha & -l - ds_\alpha & -l + ds_\beta & -l + ds_\beta
\end{bmatrix}
$$
$$
\bm{M} \bm{\dot{v}} = \bm{J}_1 \bm{\tau}
$$

伪逆 $\bm{J}_1^\dagger = \bm{J}_1^T (\bm{J}_1 \bm{J}_1^T)^{-1}$ 
$$
\bm{M} = \begin{bmatrix}
    m_1 & 0 & 0 \\
    0 & m_1 & 0 \\
    0 & 0 & J_{1E}
\end{bmatrix}
$$
$$
\bm{\tau} = \bm{J}_1^\dagger \left( \bm{M} \dot{\bm{v}} \right)
$$

## 期望轮速计算
$$
\bm{J}_2 = \frac{1}{r} \begin{bmatrix}
    c_{\alpha} & -s_{\alpha} & -l \\
    -c_{\alpha} & -s_{\alpha} & -l \\
    -c_{\beta} & s_{\beta} & -l \\
    c_{\beta} & s_{\beta} & -l
\end{bmatrix}
$$

期望轮速 $\bm{\dot{\theta}}_{ref} = \bm{J}_2 \bm{v}_{ref}$

## 二阶 LESO 设计（x, y, yaw 三轴统一）

当前控制方案中，$x, y, yaw$ 三轴均采用 **基于速度的二阶 LESO**，统一为同一结构：

- 平动方向：$v_x, v_y$
- 转动方向：$\omega$

该结构在三轴上使用一致的状态与参数形式，便于实现与调参。

---

### 1. 三轴统一二阶 LESO（基于速度）

对 $i \in \{x, y, \psi\}$，其中 $i=\psi$ 时对应角速度通道 $\omega$，建立如下模型：

$$
\dot{v}_i = b_{0,i} u_i + f_{total,i}
$$

定义扩张状态：

$$
x_1 = v_i, \quad x_2 = f_{total,i}
$$

系统表达为：

$$
\begin{cases}
\dot{\bm{x}} = \bm{Ax} + \bm{Bu} + \bm{E}\dot{f} \\
y = \bm{Cx}
\end{cases}
$$

对应矩阵为：

$$
\bm{A} = \begin{bmatrix}0 & 1 \\ 0 & 0\end{bmatrix}, \quad
\bm{B} = \begin{bmatrix}b_0 \\ 0\end{bmatrix}, \quad
\bm{E} = \begin{bmatrix}0 \\ 1\end{bmatrix}, \quad
\bm{C} = \begin{bmatrix}1 & 0\end{bmatrix}
$$

LESO 形式为：

$$
\begin{cases}
\dot{z}_1 = z_2 + l_1 (v_i - z_1) + b_{0,i} u_i \\
\dot{z}_2 = l_2 (v_i - z_1)
\end{cases}
$$

极点配置：

$$
l_1 = 2\omega_o, \quad l_2 = \omega_o^2
$$

离散化形式：

$$
e(k) = v_i(k) - z_1(k)
$$

$$
\begin{cases}
z_1(k+1) = z_1(k) + dt \cdot [z_2(k) + l_1 e(k) + b_0 u(k)] \\
z_2(k+1) = z_2(k) + dt \cdot [l_2 e(k)]
\end{cases}
$$

该二阶 LESO 结构在三轴统一使用，在保证系统鲁棒性的同时降低了实现复杂度。

## 扰动补偿控制律设计
基于 LESO 的总扰动估计，设计如下控制律：
$$ 
\bm{F}_{task} = \begin{bmatrix} 
    m_1(\dot{v}_{ref,x} + K_{v,x}(v_{ref,x} - z_{1,x}) - z_{2,x}) \\
    m_1(\dot{v}_{ref,y} + K_{v,y}(v_{ref,y} - z_{1,y}) - z_{2,y}) \\
    J_{1E}(\dot{\omega}_{ref} + K_{v,\psi}(\omega_{ref} - z_{1,\psi}) - z_{2,\psi})
\end{bmatrix} 
$$

$$
\bm{\tau}_{ff} = \bm{J}_1^\dagger \bm{F}_{task}
$$

## 驱动器 MIT 控制模式
DMH3510 驱动器支持 MIT 模式，期望电流计算公式为：
$$
i_{ref} = \frac{1}{K_{T}} (K_p (\theta_{ref} - \theta) + K_d (\dot{\theta}_{ref} - \dot{\theta}) + \tau_{ff})
$$
在实际控制过程中，我们只进行速度控制，因此将 $K_p$ 设置为 0，简化为：
$$
i_{ref} = \frac{1}{K_{T}} (K_d (\dot{\theta}_{ref} - \dot{\theta}) + \tau_{ff})
$$
在每个控制周期内，计算出 $\theta_{ref}$ 和 $\bm{\tau}_{ff}$ 后，通过 CAN 将 $K_p=0$、$K_d$、$\dot{\theta}_{ref}$ 和 $\tau_{ff}$ 发送给驱动器，再由 $\bm{u} = \bm{J}_1 (K_d (\dot{\bm{\theta}}_{ref} - \dot{\bm{\theta}}) + \bm{\tau}_{ff})$ 计算给 LESO 的输入。

