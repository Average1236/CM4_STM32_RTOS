# -*- coding: utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt
from typing import Dict, Optional, Tuple, List
from enum import Enum
import logging

# 设置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class PlanningError(Exception):
    """轨迹规划异常"""
    pass

class MotionPhase(Enum):
    """运动阶段枚举"""
    ACCEL_JERK_UP = 1
    ACCEL_CONSTANT = 2
    ACCEL_JERK_DOWN = 3
    CONSTANT_VELOCITY = 4
    DECEL_JERK_UP = 5
    DECEL_CONSTANT = 6
    DECEL_JERK_DOWN = 7
    INVALID = 0

class SCurveTrajectory:
    def __init__(self, vmax: float, amax: float, jmax: float, 
                 lambda_reduce: float = 0.99, max_iter: int = 2000):
        """
        S曲线轨迹规划器
        
        参数:
            vmax: 最大速度
            amax: 最大加速度
            jmax: 最大加加速度
            lambda_reduce: 加速度缩减因子
            max_iter: 最大迭代次数
        """
        self.vmax = float(vmax)
        self.amax = float(amax)
        self.jmax = float(jmax)
        self.lambda_reduce = float(lambda_reduce)
        self.max_iter = int(max_iter)
        self.params = None
        
        # 验证参数
        if self.vmax <= 0:
            raise ValueError("vmax must be positive")
        if self.amax <= 0:
            raise ValueError("amax must be positive")
        if self.jmax <= 0:
            raise ValueError("jmax must be positive")
        if not (0 < self.lambda_reduce < 1):
            raise ValueError("lambda_reduce must be between 0 and 1")

    def _check_feasibility(self, q0: float, q1: float, v0: float, v1: float) -> bool:
        """
        检查轨迹是否可行
        
        参数:
            q0: 起始位置
            q1: 终止位置
            v0: 起始速度
            v1: 终止速度
            
        返回:
            bool: 轨迹是否可行
        """
        # 计算最小所需位移
        dv = abs(v1 - v0)
        dq = abs(q1 - q0)
        
        # 计算达到最大加速度所需时间
        time_to_reach_max_a = self.amax / self.jmax
        time_to_set_speeds = math.sqrt(dv / self.jmax)
        
        Tj = min(time_to_reach_max_a, time_to_set_speeds)
        
        if Tj == time_to_reach_max_a:
            min_displacement = 0.5 * (v0 + v1) * (Tj + dv / self.amax)
            return dq > min_displacement
        elif Tj < time_to_reach_max_a:
            min_displacement = Tj * (v0 + v1)
            return dq > min_displacement
        
        return True

    def _sign_transform(self, q0: float, q1: float, v0: float, v1: float,
                        vmax: Optional[float] = None, amax: Optional[float] = None, jmax: Optional[float] = None
                       ) -> Tuple[float, float, float, float, float, float, float, float]:
        """
        符号变换，使运动方向始终为正
        
        参数:
            q0: 起始位置
            q1: 终止位置
            v0: 起始速度
            v1: 终止速度
            
        返回:
            Tuple: 变换后的参数
        """
        if vmax is None:
            vmax = self.vmax

        if amax is None:
            amax = self.amax

        if jmax is None:
            jmax = self.jmax

        vmin = -vmax
        amin = -amax
        jmin = -jmax

        sigma = np.sign(q1 - q0)
        if sigma == 0:
            sigma = 1.0
        vs1 = (sigma + 1) / 2
        vs2 = (sigma - 1) / 2

        q_0 = sigma * q0
        q_1 = sigma * q1
        v_0 = sigma * v0
        v_1 = sigma * v1
        v_max = vs1 * vmax + vs2 * vmin
        a_max = vs1 * amax + vs2 * amin
        j_max = vs1 * jmax + vs2 * jmin

        return q_0, q_1, v_0, v_1, v_max, a_max, j_max, sigma

    def _compute_max_speed_reached(self, q0: float, q1: float, v0: float, v1: float, 
                                  v_max: float, a_max: float, j_max: float) -> Optional[Dict]:
        """
        计算达到最大速度的情况
        
        参数:
            q0: 起始位置
            q1: 终止位置
            v0: 起始速度
            v1: 终止速度
            v_max: 最大速度
            a_max: 最大加速度
            j_max: 最大加加速度
            
        返回:
            Dict: 轨迹参数
        """
        # 加速阶段
        if (v_max - v0) * j_max < a_max ** 2:
            Tj1 = math.sqrt(max((v_max - v0) / j_max, 0.0))
            Ta = 2.0 * Tj1
            a_lima = j_max * Tj1
        else:
            Tj1 = a_max / j_max
            Ta = Tj1 + (v_max - v0) / a_max
            a_lima = a_max

        # 减速阶段
        if (v_max - v1) * j_max < a_max ** 2:
            Tj2 = math.sqrt(max((v_max - v1) / j_max, 0.0))
            Td = 2.0 * Tj2
            a_limd = -j_max * Tj2
        else:
            Tj2 = a_max / j_max
            Td = Tj2 + (v_max - v1) / a_max
            a_limd = -a_max

        # 计算匀速阶段时间
        if abs(v_max) < 1e-12:
            Tv = -1.0
        else:
            Tv = (q1 - q0) / v_max - (Ta / 2.0) * (1.0 + v0 / v_max) - (Td / 2.0) * (1.0 + v1 / v_max)

        if Tv > 0:
            vlim = v_max
            return {
                'Ta': Ta, 'Tv': Tv, 'Td': Td, 'Tj1': Tj1, 'Tj2': Tj2,
                'q0': q0, 'q1': q1, 'v0': v0, 'v1': v1, 'vlim': vlim,
                'amax': a_max, 'amin': -a_max, 'alima': a_lima, 'alimd': a_limd,
                'jmax': j_max, 'jmin': -j_max, 'sigma': 1.0
            }
        
        return None

    def _compute_max_speed_not_reached(self, q0: float, q1: float, v0: float, v1: float, 
                                      v_max: float, a_max: float, j_max: float) -> Optional[Dict]:
        """
        计算未达到最大速度的情况
        
        参数:
            q0: 起始位置
            q1: 终止位置
            v0: 起始速度
            v1: 终止速度
            v_max: 最大速度
            a_max: 最大加速度
            j_max: 最大加加速度
            
        返回:
            Dict: 轨迹参数
        """
        Tv = 0.0
        Tj = a_max / j_max
        Tj1 = Tj2 = Tj

        # 计算判别式
        delta = ((a_max ** 4) / (j_max ** 2) + 2.0 * (v0 ** 2 + v1 ** 2) +
                a_max * (4.0 * (q1 - q0) - 2.0 * (a_max / j_max) * (v0 + v1)))

        if delta < 0 and abs(delta) < 1e-12:
            delta = 0.0

        sqrt_delta = math.sqrt(max(delta, 0.0))
        
        # 计算加速和减速时间
        if a_max != 0:
            Ta = ((a_max ** 2 / j_max) - 2.0 * v0 + sqrt_delta) / (2.0 * a_max)
            Td = ((a_max ** 2 / j_max) - 2.0 * v1 + sqrt_delta) / (2.0 * a_max)
        else:
            Ta = Td = 0.0

        # 处理负时间情况
        if Ta < 0 or Td < 0:
            if Ta < 0:
                # 只有减速阶段
                Ta = 0.0
                Tj1 = 0.0
                if (v0 + v1) != 0:
                    Td = 2.0 * (q1 - q0) / (v0 + v1)
                else:
                    Td = 0.0
                
                under = j_max * (j_max * (q1 - q0) ** 2 + (v1 + v0) ** 2 * (v1 - v0))
                under = max(under, 0.0)
                
                if j_max * (v1 + v0) != 0:
                    Tj2 = (j_max * (q1 - q0) - math.sqrt(under)) / (j_max * (v1 + v0))
                else:
                    Tj2 = 0.0
                
                a_lima = 0.0
                a_limd = -j_max * Tj2
                vlim = v0
                
                return {
                    'Ta': Ta, 'Tv': Tv, 'Td': Td, 'Tj1': Tj1, 'Tj2': Tj2,
                    'q0': q0, 'q1': q1, 'v0': v0, 'v1': v1, 'vlim': vlim,
                    'amax': a_max, 'amin': -a_max, 'alima': a_lima, 'alimd': a_limd,
                    'jmax': j_max, 'jmin': -j_max, 'sigma': 1.0
                }
            elif Td < 0:
                # 只有加速阶段
                Td = 0.0
                Tj2 = 0.0
                if (v0 + v1) != 0:
                    Ta = 2.0 * (q1 - q0) / (v0 + v1)
                else:
                    Ta = 0.0
                
                under = j_max * (j_max * (q1 - q0) ** 2) - (v1 + v0) ** 2 * (v1 - v0)
                under = max(under, 0.0)
                
                if j_max * (v1 + v0) != 0:
                    Tj1 = (j_max * (q1 - q0) - math.sqrt(under)) / (j_max * (v1 + v0))
                else:
                    Tj1 = 0.0
                
                a_lima = j_max * Tj1
                a_limd = 0.0
                vlim = v0 + a_lima * (Ta - Tj1)
                
                return {
                    'Ta': Ta, 'Tv': Tv, 'Td': Td, 'Tj1': Tj1, 'Tj2': Tj2,
                    'q0': q0, 'q1': q1, 'v0': v0, 'v1': v1, 'vlim': vlim,
                    'amax': a_max, 'amin': -a_max, 'alima': a_lima, 'alimd': a_limd,
                    'jmax': j_max, 'jmin': -j_max, 'sigma': 1.0
                }

        # 检查是否达到最大加速度
        if Ta >= 2.0 * Tj and Td >= 2.0 * Tj:
            a_lima = a_max
            a_limd = -a_max
            vlim = v0 + a_lima * (Ta - Tj)
            
            return {
                'Ta': Ta, 'Tv': Tv, 'Td': Td, 'Tj1': Tj1, 'Tj2': Tj2,
                'q0': q0, 'q1': q1, 'v0': v0, 'v1': v1, 'vlim': vlim,
                'amax': a_max, 'amin': -a_max, 'alima': a_lima, 'alimd': a_limd,
                'jmax': j_max, 'jmin': -j_max, 'sigma': 1.0
            }
        
        return None

    def _search_planning(self, q0: float, q1: float, v0: float, v1: float, 
                        v_max: float, a_max: float, j_max: float) -> Dict:
        """
        搜索可行的轨迹参数
        
        参数:
            q0: 起始位置
            q1: 终止位置
            v0: 起始速度
            v1: 终止速度
            v_max: 最大速度
            a_max: 最大加速度
            j_max: 最大加加速度
            
        返回:
            Dict: 轨迹参数
        """
        iter_count = 0
        current_amax = a_max
        
        while iter_count < self.max_iter and current_amax > 1e-12:
            iter_count += 1
            current_amax *= self.lambda_reduce
            
            # 尝试计算参数
            params = self._compute_max_speed_not_reached(q0, q1, v0, v1, v_max, current_amax, j_max)
            
            if params is not None:
                return params
        
        # 最终回退方案
        Tj = current_amax / j_max
        Tj1 = Tj2 = Tj
        Ta = Td = 0.0
        
        # 计算判别式
        delta = ((current_amax ** 4) / (j_max ** 2) + 2.0 * (v0 ** 2 + v1 ** 2) +
                current_amax * (4.0 * (q1 - q0) - 2.0 * (current_amax / j_max) * (v0 + v1)))
        
        if delta < 0 and abs(delta) < 1e-12:
            delta = 0.0
        
        sqrt_delta = math.sqrt(max(delta, 0.0))
        
        if current_amax != 0:
            Ta = ((current_amax ** 2 / j_max) - 2.0 * v0 + sqrt_delta) / (2.0 * current_amax)
            Td = ((current_amax ** 2 / j_max) - 2.0 * v1 + sqrt_delta) / (2.0 * current_amax)
        
        a_lima = current_amax
        a_limd = -current_amax
        vlim = v0 + a_lima * max(Ta - Tj, 0.0)
        
        return {
            'Ta': Ta, 'Tv': 0.0, 'Td': Td, 'Tj1': Tj1, 'Tj2': Tj2,
            'q0': q0, 'q1': q1, 'v0': v0, 'v1': v1, 'vlim': vlim,
            'amax': current_amax, 'amin': -current_amax, 'alima': a_lima, 'alimd': a_limd,
            'jmax': j_max, 'jmin': -j_max, 'sigma': 1.0
        }

    def compute_params(self, q0: float, q1: float, v0: float = 0.0, v1: float = 0.0, 
                       vmax: Optional[float] = None, amax: Optional[float] = None, jmax: Optional[float] = None) -> Dict:
        """
        计算S曲线轨迹参数
        
        参数:
            q0: 起始位置
            q1: 终止位置
            v0: 起始速度
            v1: 终止速度
            vmax: 最大速度
            amax: 最大加速度
            jmax: 最大加加速度

        返回:
            Dict: 轨迹参数
            
        异常:
            PlanningError: 当轨迹不可行时抛出
        """
        if vmax is None:
            vmax = self.vmax
        if amax is None:
            amax = self.amax
        if jmax is None:
            jmax = self.jmax

        # 符号变换
        q_0, q_1, v_0, v_1, v_max, a_max, j_max, sigma = self._sign_transform(q0, q1, v0, v1, vmax, amax, jmax)

        # 检查轨迹可行性
        if not self._check_feasibility(q_0, q_1, v_0, v_1):
            raise PlanningError("Trajectory is not feasible with given constraints")
        
        
        
        # 尝试计算达到最大速度的情况
        params = self._compute_max_speed_reached(q_0, q_1, v_0, v_1, v_max, a_max, j_max)
        
        if params is not None:
            params['sigma'] = sigma
            self.params = params
            return params
        
        # 尝试计算未达到最大速度的情况
        params = self._compute_max_speed_not_reached(q_0, q_1, v_0, v_1, v_max, a_max, j_max)
        
        if params is not None:
            params['sigma'] = sigma
            self.params = params
            return params
        
        # 搜索可行的轨迹参数
        params = self._search_planning(q_0, q_1, v_0, v_1, v_max, a_max, j_max)
        params['sigma'] = sigma
        self.params = params
        
        return params

    def compute_params_nD(self, q0, q1, v0=None, v1=None, vmax=None, amax=None, jmax=None, t=None):
        """
        计算多维度S曲线轨迹参数
        
        参数:
            q0: 起始位置数组 (ndof,)
            q1: 终止位置数组 (ndof,)
            v0: 起始速度数组 (ndof,), 默认为零
            v1: 终止速度数组 (ndof,), 默认为零
            vmax: 最大速度 (标量或数组)
            amax: 最大加速度 (标量或数组)
            jmax: 最大加加速度 (标量或数组)
            t: 指定的总时间，如果提供则进行时间同步

        返回:
            Dict: 多维度轨迹参数，包含每个自由度的参数
            
        异常:
            PlanningError: 当轨迹不可行时抛出
        """
        # 转换为numpy数组并检查形状
        q0 = np.asarray(q0, dtype=float)
        q1 = np.asarray(q1, dtype=float)
        
        if q0.shape != q1.shape:
            raise ValueError("q0 and q1 must have the same shape")
        
        ndof = q0.shape[0] if q0.ndim == 1 else q0.shape[0]
        
        # 处理默认值
        if v0 is None:
            v0 = np.zeros(ndof)
        else:
            v0 = np.asarray(v0, dtype=float)
            
        if v1 is None:
            v1 = np.zeros(ndof)
        else:
            v1 = np.asarray(v1, dtype=float)
            
        if vmax is None:
            vmax = np.full(ndof, self.vmax)
        elif np.isscalar(vmax):
            vmax = np.full(ndof, vmax)
        else:
            vmax = np.asarray(vmax, dtype=float)
            
        if amax is None:
            amax = np.full(ndof, self.amax)
        elif np.isscalar(amax):
            amax = np.full(ndof, amax)
        else:
            amax = np.asarray(amax, dtype=float)
            
        if jmax is None:
            jmax = np.full(ndof, self.jmax)
        elif np.isscalar(jmax):
            jmax = np.full(ndof, jmax)
        else:
            jmax = np.asarray(jmax, dtype=float)

        logger.info("\r\n********************************************"
                   "\r\n\t"
                   "NEW MULTI-DOF TRAJECTORY\r\n"
                   "********************************************")

        # 计算位移并找到最长轨迹的自由度
        dq = q1 - q0
        max_displacement_id = np.argmax(np.abs(dq))
        
        logger.info(f"Computing the longest DOF trajectory with id {max_displacement_id}")

        # 存储每个自由度的参数
        all_params = {}
        trajectory_times = np.zeros(ndof)

        # 首先计算最长轨迹的参数
        max_dof_params = self.compute_params(
            q0[max_displacement_id], 
            q1[max_displacement_id],
            v0[max_displacement_id], 
            v1[max_displacement_id],
            vmax[max_displacement_id], 
            amax[max_displacement_id], 
            jmax[max_displacement_id]
        )
        
        all_params[max_displacement_id] = max_dof_params
        max_time = max_dof_params['Ta'] + max_dof_params['Tv'] + max_dof_params['Td']
        trajectory_times[max_displacement_id] = max_time
        
        # 如果指定了总时间，使用指定时间
        if t is not None:
            sync_time = float(t)
        else:
            sync_time = max_time

        # 计算其他自由度的参数
        for dof in range(ndof):
            if dof == max_displacement_id:
                continue
                
            logger.info(f"Computing {dof} DOF trajectory")
            
            try:
                # 如果终止速度非零，需要进行时间同步
                if abs(v1[dof]) > 1e-12:
                    # 尝试使用同步时间进行规划
                    dof_params = self._compute_params_with_time_constraint(
                        q0[dof], q1[dof], v0[dof], v1[dof],
                        vmax[dof], amax[dof], jmax[dof], sync_time
                    )
                else:
                    # 终止速度为零时，直接计算最优参数
                    dof_params = self.compute_params(
                        q0[dof], q1[dof], v0[dof], v1[dof],
                        vmax[dof], amax[dof], jmax[dof]
                    )
                
                all_params[dof] = dof_params
                dof_time = dof_params['Ta'] + dof_params['Tv'] + dof_params['Td']
                trajectory_times[dof] = dof_time
                
            except PlanningError as e:
                logger.warning(f"Planning failed for DOF {dof}: {e}")
                # 使用线性插值作为后备方案
                dof_params = self._create_linear_fallback_params(
                    q0[dof], q1[dof], v0[dof], v1[dof], sync_time
                )
                all_params[dof] = dof_params
                trajectory_times[dof] = sync_time

        # 构建返回结果
        result = {
            'ndof': ndof,
            'sync_time': sync_time,
            'max_displacement_dof': max_displacement_id,
            'trajectory_times': trajectory_times,
            'params_per_dof': all_params,
            'q0': q0,
            'q1': q1,
            'v0': v0,
            'v1': v1
        }
        
        self.params_nD = result
        return result

    def _compute_params_with_time_constraint(self, q0, q1, v0, v1, vmax, amax, jmax, target_time, max_iter: Optional[int] = None):
        """
        在给定时间约束下计算轨迹参数
        
        参数:
            q0, q1: 起止位置
            v0, v1: 起止速度
            vmax, amax, jmax: 约束参数
            target_time: 目标时间
            
        返回:
            Dict: 轨迹参数
        """
        logger.info("Starting search planning with time constraint")

        if max_iter is None:
            max_iter = self.max_iter
        _amax = amax
        it = 0

        while (it < max_iter) and (_amax > 1e-4):
            try:
                params = self.compute_params(q0, q1, v0, v1, vmax, _amax, jmax)
                total_time = params['Ta'] + params['Tv'] + params['Td']
                
                if abs(total_time - target_time) <= 1e-2:
                    return params
                else:
                    # 时间太短，减小加速度
                    _amax *= self.lambda_reduce
                    it += 1
                
            except PlanningError:
                # 规划失败，减小加速度
                _amax *= self.lambda_reduce
                it += 1
        
        raise PlanningError(f"Cannot satisfy time constraint {target_time} for trajectory from {q0} to {q1}")

    def _create_linear_fallback_params(self, q0, q1, v0, v1, target_time):
        """
        创建线性插值的后备轨迹参数
        
        参数:
            q0, q1: 起止位置
            v0, v1: 起止速度 
            target_time: 目标时间
            
        返回:
            Dict: 简化的轨迹参数
        """
        return {
            'Ta': target_time / 3.0,
            'Tv': target_time / 3.0, 
            'Td': target_time / 3.0,
            'Tj1': target_time / 6.0,
            'Tj2': target_time / 6.0,
            'q0': q0,
            'q1': q1,
            'v0': v0,
            'v1': v1,
            'vlim': (q1 - q0) / target_time + (v0 + v1) / 2.0,
            'amax': self.amax * 0.1,
            'amin': -self.amax * 0.1,
            'alima': self.amax * 0.1,
            'alimd': -self.amax * 0.1,
            'jmax': self.jmax * 0.1,
            'jmin': -self.jmax * 0.1,
            'sigma': 1.0
        }

    def position_nD(self, t: float, dof: Optional[int] = None):
        """
        计算多维度轨迹在时间t的位置
        
        参数:
            t: 时间
            dof: 自由度索引，如果为None则返回所有自由度
            
        返回:
            float或np.ndarray: 位置值
        """
        if not hasattr(self, 'params_nD') or self.params_nD is None:
            raise ValueError("Call compute_params_nD() first")
        
        params_dict = self.params_nD
        
        if dof is not None:
            # 返回单个自由度的位置
            if dof not in params_dict['params_per_dof']:
                raise ValueError(f"DOF {dof} not found")
            return self.position_scalar(t, params_dict['params_per_dof'][dof])
        else:
            # 返回所有自由度的位置
            positions = []
            for i in range(params_dict['ndof']):
                pos = self.position_scalar(t, params_dict['params_per_dof'][i])
                positions.append(pos)
            return np.array(positions)

    def velocity_nD(self, t: float, dof: Optional[int] = None):
        """
        计算多维度轨迹在时间t的速度
        
        参数:
            t: 时间
            dof: 自由度索引，如果为None则返回所有自由度
            
        返回:
            float或np.ndarray: 速度值
        """
        if not hasattr(self, 'params_nD') or self.params_nD is None:
            raise ValueError("Call compute_params_nD() first")
        
        params_dict = self.params_nD
        
        if dof is not None:
            if dof not in params_dict['params_per_dof']:
                raise ValueError(f"DOF {dof} not found")
            return self.velocity_scalar(t, params_dict['params_per_dof'][dof])
        else:
            velocities = []
            for i in range(params_dict['ndof']):
                vel = self.velocity_scalar(t, params_dict['params_per_dof'][i])
                velocities.append(vel)
            return np.array(velocities)

    def acceleration_nD(self, t: float, dof: Optional[int] = None):
        """
        计算多维度轨迹在时间t的加速度
        
        参数:
            t: 时间
            dof: 自由度索引，如果为None则返回所有自由度
            
        返回:
            float或np.ndarray: 加速度值
        """
        if not hasattr(self, 'params_nD') or self.params_nD is None:
            raise ValueError("Call compute_params_nD() first")
        
        params_dict = self.params_nD
        
        if dof is not None:
            if dof not in params_dict['params_per_dof']:
                raise ValueError(f"DOF {dof} not found")
            return self.acceleration_scalar(t, params_dict['params_per_dof'][dof])
        else:
            accelerations = []
            for i in range(params_dict['ndof']):
                acc = self.acceleration_scalar(t, params_dict['params_per_dof'][i])
                accelerations.append(acc)
            return np.array(accelerations)

    def get_motion_phase(self, t: float) -> MotionPhase:
        """
        获取指定时间的运动阶段
        
        参数:
            t: 时间
            
        返回:
            MotionPhase: 运动阶段
        """
        if self.params is None:
            return MotionPhase.INVALID
        
        p = self.params
        Ta, Tv, Td = p['Ta'], p['Tv'], p['Td']
        Tj1, Tj2 = p['Tj1'], p['Tj2']
        T = Ta + Tv + Td
        
        if t < 0:
            return MotionPhase.INVALID
        elif t < Tj1:
            return MotionPhase.ACCEL_JERK_UP
        elif t < Ta - Tj1:
            return MotionPhase.ACCEL_CONSTANT
        elif t < Ta:
            return MotionPhase.ACCEL_JERK_DOWN
        elif t < Ta + Tv:
            return MotionPhase.CONSTANT_VELOCITY
        elif t < T - Td + Tj2:
            return MotionPhase.DECEL_JERK_UP
        elif t < T - Tj2:
            return MotionPhase.DECEL_CONSTANT
        elif t <= T:
            return MotionPhase.DECEL_JERK_DOWN
        else:
            return MotionPhase.INVALID

    def position(self, t: float) -> float:
        """
        计算位置
        
        参数:
            t: 时间
            
        返回:
            float: 位置
        """
        return self.position_scalar(t)

    def velocity(self, t: float) -> float:
        """
        计算速度
        
        参数:
            t: 时间
            
        返回:
            float: 速度
        """
        return self.velocity_scalar(t)

    def acceleration(self, t: float) -> float:
        """
        计算加速度
        
        参数:
            t: 时间
            
        返回:
            float: 加速度
        """
        return self.acceleration_scalar(t)

    def jerk(self, t: float) -> float:
        """
        计算加加速度
        
        参数:
            t: 时间
            
        返回:
            float: 加加速度
        """
        return self.jerk_scalar(t)

    def position_scalar(self, t: float, p: Optional[Dict] = None) -> float:
        if p is None:
            p = self.params
            if p is None:
                raise ValueError("No parameters available. Call compute_params() first.")
        
        Ta, Tv, Td = p['Ta'], p['Tv'], p['Td']
        Tj1, Tj2 = p['Tj1'], p['Tj2']
        q0, q1, v0, v1, vlim = p['q0'], p['q1'], p['v0'], p['v1'], p['vlim']
        alima, alimd = p['alima'], p['alimd']
        jmax, jmin = p['jmax'], p['jmin']
        T = Ta + Tv + Td
        
        # 计算匀速段开始时的位置（加速段终点）
        q_acc_end = q0 + (vlim + v0) * Ta / 2.0
        # 计算匀速段结束时的位置（减速段起点）
        q_dec_start = q1 - (vlim + v1) * Td / 2.0
        
        if t <= 0.0:
            return q0 * p['sigma']
        if t < Tj1:
            result = q0 + v0 * t + jmax * (t ** 3) / 6.0
        elif t < Ta - Tj1:
            result = q0 + v0 * t + (alima / 6.0) * (3.0 * (t ** 2) - 3.0 * Tj1 * t + (Tj1 ** 2))
        elif t < Ta:
            result = q0 + (vlim + v0) * (Ta / 2.0) - vlim * (Ta - t) - jmin * (((Ta - t) ** 3) / 6.0)
        elif t <= Ta + Tv:
            result = q0 + (vlim + v0) * Ta / 2.0 + vlim * (t - Ta)  # 从加速段终点正向计算
        else:
            # 减速段统一从匀速段终点正向计算
            tt = t - Ta - Tv  # 减速段相对时间
            
            if tt < Tj2:
                result = q_dec_start + vlim * tt - jmax * (tt ** 3) / 6.0
            elif tt < Td - Tj2:
                result = q_dec_start + vlim * tt + (alimd / 6.0) * (3.0 * (tt ** 2) - 3.0 * Tj2 * tt + (Tj2 ** 2))
            elif t <= T:
                tt_end = T - t
                result = q1 - v1 * tt_end - jmax * (tt_end ** 3) / 6.0
            else:
                result = q1
        
        return result * p['sigma']

    def velocity_scalar(self, t: float, p: Optional[Dict] = None) -> float:
        """
        标量速度计算
        
        参数:
            t: 时间
            p: 轨迹参数
            
        返回:
            float: 速度
        """
        if p is None:
            p = self.params
            if p is None:
                raise ValueError("No parameters available. Call compute_params() first.")
        
        Ta, Tv, Td = p['Ta'], p['Tv'], p['Td']
        Tj1, Tj2 = p['Tj1'], p['Tj2']
        v0, v1, vlim = p['v0'], p['v1'], p['vlim']
        alima, alimd = p['alima'], p['alimd']
        jmax, jmin = p['jmax'], p['jmin']
        T = Ta + Tv + Td

        if t <= 0.0:
            return v0 * p['sigma']  # 恢复原始符号
        if t < Tj1:
            result = v0 + jmax * (t ** 2) / 2.0
        elif t < Ta - Tj1:
            result = v0 + alima * (t - Tj1 / 2.0)
        elif t < Ta:
            result = vlim + jmin * (((Ta - t) ** 2) / 2.0)
        elif t < Ta + Tv:
            result = vlim
        elif t < T - Td + Tj2:
            tt = t - T + Td
            result = vlim - jmax * ((tt ** 2) / 2.0)
        elif t < T - Tj2:
            tt = t - T + Td
            result = vlim + alimd * (tt - Tj2 / 2.0)
        elif t <= T:
            result = v1 + jmax * (((t - T) ** 2) / 2.0)
        else:
            result = v1
        
        return result * p['sigma']  # 恢复原始符号

    def acceleration_scalar(self, t: float, p: Optional[Dict] = None) -> float:
        """
        标量加速度计算
        
        参数:
            t: 时间
            p: 轨迹参数
            
        返回:
            float: 加速度
        """
        if p is None:
            p = self.params
            if p is None:
                raise ValueError("No parameters available. Call compute_params() first.")
        
        Ta, Tv, Td = p['Ta'], p['Tv'], p['Td']
        Tj1, Tj2 = p['Tj1'], p['Tj2']
        alima, alimd = p['alima'], p['alimd']
        jmax, jmin = p['jmax'], p['jmin']
        T = Ta + Tv + Td

        if t <= 0.0:
            return 0.0
        if t < Tj1:
            result = jmax * t
        elif t < Ta - Tj1:
            result = alima
        elif t < Ta:
            result = -jmin * (Ta - t)
        elif t < Ta + Tv:
            result = 0.0
        elif t < T - Td + Tj2:
            tt = t - T + Td
            result = -jmax * tt
        elif t < T - Tj2:
            tt = t - T + Td
            result = alimd
        elif t <= T:
            result = -jmax * (T - t)
        else:
            result = 0.0
        
        return result * p['sigma']  # 恢复原始符号

    def jerk_scalar(self, t: float, p: Optional[Dict] = None) -> float:
        """
        标量加加速度计算
        
        参数:
            t: 时间
            p: 轨迹参数
            
        返回:
            float: 加加速度
        """
        if p is None:
            p = self.params
            if p is None:
                raise ValueError("No parameters available. Call compute_params() first.")
        
        Ta, Tv, Td = p['Ta'], p['Tv'], p['Td']
        Tj1, Tj2 = p['Tj1'], p['Tj2']
        jmax, jmin = p['jmax'], p['jmin']
        T = Ta + Tv + Td

        if t <= 0.0:
            return 0.0
        if t < Tj1:
            result = jmax
        elif t < Ta - Tj1:
            result = 0.0
        elif t < Ta:
            result = jmin
        elif t < Ta + Tv:
            result = 0.0
        elif t < T - Td + Tj2:
            result = -jmax
        elif t < T - Tj2:
            result = 0.0
        elif t <= T:
            result = jmax
        else:
            result = 0.0
        
        return result * p['sigma']  # 恢复原始符号

    def normalization_S(self, pos: float, v0: float, v1: float, t: float
                       ) -> Tuple[np.ndarray, int]:
        """
        S型速度规划的归一化
        
        参数:
            pos: 位移
            v0: 起始速度
            v1: 终止速度
            t: 时间间隔
            
        返回:
            Tuple: 归一化位移数组和插值点数
        """
        if abs(pos) < 1e-12:
            # 返回包含单个点的数组
            return np.array([0.0]), 1

        try:
            para = self.compute_params(0.0, pos, v0, v1)
            T = float(para['Ta'] + para['Tv'] + para['Td'])
            
            # 等时插补
            n_samples = max(2, int(math.ceil(T / t)) + 1)  # 至少2个点
            times = np.linspace(0.0, T, n_samples)
            
            lambda_list = []
            for ti in times:
                q_val = self.position_scalar(ti, para)
                lambda_list.append(float(q_val) / float(pos))
            
            lambda_arr = np.asarray(lambda_list, dtype=float)
            return lambda_arr, int(len(lambda_arr))
        
        except PlanningError:
            # 规划失败时返回线性插值
            n_samples = max(2, int(math.ceil(abs(pos) / (max(abs(v0), abs(v1)) * t)) + 1))
            return np.linspace(0.0, 1.0, n_samples), n_samples

    def normalization_S_nD(self, pos: np.ndarray, v0: np.ndarray,
                            v1: np.ndarray, t: float) -> Tuple[np.ndarray, int]:
        """
        S型速度规划的归一化（多维版本）

        参数:
            pos: 位移（ndarray）
            v0: 起始速度（ndarray）
            v1: 终止速度（ndarray）
            t: 插值周期

        返回:
            Tuple: 归一化位移数组和插值点数
        """
        pos = np.asarray(pos, dtype=float)
        v0 = np.asarray(v0, dtype=float)
        v1 = np.asarray(v1, dtype=float)

        try:
            para = self.compute_params_nD(0.0 * pos, pos, v0, v1)
            T = para['sync_time']
            
            # 等时插补
            n_samples = max(2, int(math.ceil(T / t)) + 1)  # 至少2个点
            times = np.linspace(0.0, T, n_samples)
            
            lambda_list = []
            for ti in times:
                q_val = self.position_nD(ti)
                lambda_list.append(q_val / pos)
            
            lambda_arr = np.asarray(lambda_list, dtype=float)
            return lambda_arr, int(len(lambda_arr))
        except PlanningError:
            # 规划失败时返回线性插值
            logger.warning("Multi-DOF trajectory planning failed, using linear interpolation")
            max_v = np.maximum(np.abs(v0), np.abs(v1))
            max_v[max_v < 1e-12] = 1e-12  # 防止除零
            n_samples = max(2, int(math.ceil(np.max(np.abs(pos)) / (np.max(max_v) * t)) + 1))
            return np.tile(np.linspace(0.0, 1.0, n_samples)[:, None], (1, pos.shape[0])), n_samples

    def evaluate(self, t_samples: Optional[np.ndarray] = None, n_samples: int = 600
                ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        评估轨迹
        
        参数:
            t_samples: 时间样本数组
            n_samples: 样本数量
            
        返回:
            Tuple: 时间、位置、速度、加速度、加加速度数组
        """
        if self.params is None:
            raise ValueError("Call compute_params() before evaluate")
        
        p = self.params
        Ta, Tv, Td = p['Ta'], p['Tv'], p['Td']
        T_total = Ta + Tv + Td
        
        if T_total <= 0:
            T_total = 1.0
        
        if t_samples is None:
            t_samples = np.linspace(0.0, T_total, n_samples)
        
        # 向量化计算
        q = np.vectorize(self.position)(t_samples)
        qd = np.vectorize(self.velocity)(t_samples)
        qdd = np.vectorize(self.acceleration)(t_samples)
        qddd = np.vectorize(self.jerk)(t_samples)
        
        return t_samples, q, qd, qdd, qddd

    def plot(self, t_samples: Optional[np.ndarray] = None, n_samples: int = 600, 
             show: bool = True, figsize: Tuple[int, int] = (10, 8)) -> None:
        """
        绘制轨迹图形
        
        参数:
            t_samples: 时间样本数组
            n_samples: 样本数量
            show: 是否显示图形
            figsize: 图形大小
        """
        t, q, qd, qdd, qddd = self.evaluate(t_samples=t_samples, n_samples=n_samples)
        
        plt.figure(figsize=figsize)
        
        plt.subplot(4, 1, 1)
        plt.plot(t, q)
        plt.title("Position")
        plt.xlabel("Time (s)")
        plt.ylabel("Position")
        plt.grid(True)
        
        plt.subplot(4, 1, 2)
        plt.plot(t, qd)
        plt.title("Velocity")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity")
        plt.grid(True)
        
        plt.subplot(4, 1, 3)
        plt.plot(t, qdd)
        plt.title("Acceleration")
        plt.xlabel("Time (s)")
        plt.ylabel("Acceleration")
        plt.grid(True)
        
        plt.subplot(4, 1, 4)
        plt.plot(t, qddd)
        plt.title("Jerk")
        plt.xlabel("Time (s)")
        plt.ylabel("Jerk")
        plt.grid(True)
        
        plt.tight_layout()
        
        if show:
            plt.show()

    def get_total_time(self) -> float:
        """
        获取总运动时间
        
        返回:
            float: 总时间
        """
        if self.params is None:
            raise ValueError("No parameters available. Call compute_params() first.")
        
        return self.params['Ta'] + self.params['Tv'] + self.params['Td']

    def get_summary(self) -> Dict:
        """
        获取轨迹摘要信息
        
        返回:
            Dict: 摘要信息
        """
        if self.params is None:
            raise ValueError("No parameters available. Call compute_params() first.")
        
        p = self.params
        T_total = p['Ta'] + p['Tv'] + p['Td']
        
        return {
            'total_time': T_total,
            'acceleration_time': p['Ta'],
            'constant_velocity_time': p['Tv'],
            'deceleration_time': p['Td'],
            'jerk_up_time': p['Tj1'],
            'jerk_down_time': p['Tj2'],
            'max_velocity': p['vlim'] * p['sigma'],
            'max_acceleration': p['alima'] * p['sigma'],
            'max_deceleration': p['alimd'] * p['sigma']
        }


if __name__ == "__main__":    
    try:
        # 单维度测试
        print("=== 单维度S曲线轨迹测试 ===")
        traj = SCurveTrajectory(vmax=1.0, amax=2.0, jmax=10.0)
        params = traj.compute_params(q0=0.0, q1=0.9615673578611098, v0=0.0, v1=0.06349494314662162)
        
        print("计算参数:")
        for k, v in params.items():
            print(f"  {k}: {v}")
        
        print("\n轨迹摘要:")
        summary = traj.get_summary()
        for k, v in summary.items():
            print(f"  {k}: {v}")
        
        # 多维度测试
        print("\n=== 多维度S曲线轨迹测试 ===")
        traj_nD = SCurveTrajectory(vmax=1.0, amax=2.0, jmax=10.0)
        
        # 3自由度测试
        q0_nD = np.array([0.0, 0.0, 0.0])
        q1_nD = np.array([0.9615673578611098, -0.5000150260877771, 1.0577240936472208]) 
        v0_nD = np.array([0.0, 0.0, 0.0])
        v1_nD = np.array([0.06349494314662162, -0.03301737043624324, 0.06984443746128377])
        
        params_nD = traj_nD.compute_params_nD(q0_nD, q1_nD, v0_nD, v1_nD)
        
        print(f"多维度轨迹参数:")
        print(f"  自由度数: {params_nD['ndof']}")
        print(f"  同步时间: {params_nD['sync_time']:.4f}s")
        print(f"  最长位移自由度: {params_nD['max_displacement_dof']}")
        
        # # 测试轨迹评估
        # test_times = np.array([0.0, 0.1, 0.2, 0.3])
        # print(f"\n轨迹评估测试:")
        # for t in test_times:
        #     pos = traj_nD.position_nD(t)
        #     vel = traj_nD.velocity_nD(t)
        #     acc = traj_nD.acceleration_nD(t) 
        #     print(f"  t={t:.1f}s: pos={pos}, vel={vel}, acc={acc}")
        
        # # 单个自由度测试
        # print(f"\n单个自由度测试 (DOF 0):")
        # for t in test_times:
        #     pos = traj_nD.position_nD(t, dof=0)
        #     vel = traj_nD.velocity_nD(t, dof=0)
        #     print(f"  t={t:.1f}s: pos={pos:.4f}, vel={vel:.4f}")
        
        # 可视化单维度轨迹
        traj.plot(n_samples=800)

        # 可视化多维度轨迹
        print("\n=== 多维度轨迹可视化 ===")
        t_eval = np.linspace(0, params_nD['sync_time'], 800)
        
        # 计算所有自由度的轨迹
        pos_all = []
        vel_all = []
        acc_all = []
        
        for t in t_eval:
            pos_all.append(traj_nD.position_nD(t))
            vel_all.append(traj_nD.velocity_nD(t))
            acc_all.append(traj_nD.acceleration_nD(t))
        
        pos_all = np.array(pos_all)
        vel_all = np.array(vel_all)
        acc_all = np.array(acc_all)
        
        # 绘制多关节轨迹图
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        
        # 位置曲线
        axes[0].set_title("Multi-DOF Position Trajectories")
        for dof in range(params_nD['ndof']-1):
            axes[0].plot(t_eval, pos_all[:, dof], label=f'Joint {dof}', linewidth=2)
        axes[0].set_xlabel("Time (s)")
        axes[0].set_ylabel("Position")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        
        # 速度曲线
        axes[1].set_title("Multi-DOF Velocity Trajectories")
        for dof in range(params_nD['ndof']-1):
            axes[1].plot(t_eval, vel_all[:, dof], label=f'Joint {dof}', linewidth=2)
        axes[1].set_xlabel("Time (s)")
        axes[1].set_ylabel("Velocity")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        
        # 加速度曲线
        axes[2].set_title("Multi-DOF Acceleration Trajectories")
        for dof in range(params_nD['ndof']-1):
            axes[2].plot(t_eval, acc_all[:, dof], label=f'Joint {dof}', linewidth=2)
        axes[2].set_xlabel("Time (s)")
        axes[2].set_ylabel("Acceleration")
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
        
    except PlanningError as e:
        print(f"轨迹规划错误: {e}")
    except ValueError as e:
        print(f"参数错误: {e}")