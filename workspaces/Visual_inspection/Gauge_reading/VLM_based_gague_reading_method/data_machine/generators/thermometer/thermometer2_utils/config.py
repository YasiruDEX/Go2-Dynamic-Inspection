# Thermometer/config.py

import random
import numpy as np

# --- 可配置的策略库 ---

# 1. 取值范围: [摄氏度最小值, 摄氏度最大值, 最小刻度, 大刻度间距]
RANGES_C = [
    [-20, 110, 1, 10],
    [-10, 110, 1, 10],
    [-5, 10, 0.1, 1],
    [0, 5, 0.1, 1],
    [-30, 50, 1, 10],
    [0, 40, 1, 10],
    [-70, 40, 2, 10],
    [35, 42, 0.1, 1],
    [-3, 40, 1, 10],
    [-3, 50, 1, 10],
    [0, 50, 1, 10],
    [-50, 50, 1, 10],
    [0, 100, 1, 10],
]

# 2. 形状/背景
SHAPES = [
    "on_glass",  # 刻度在玻璃管上
    "sealed_in_tube",  # 毛细管和刻度标尺一同熔封在更粗的玻璃管内
    "on_backplate",  # 玻璃管固定在较大的背板上
]

# 3. 刻度类型: 单刻度或双刻度
SCALE_TYPES = ["C", "F", "C_F"]  # 摄氏, 华氏, 或两者都有

# 4. 液柱颜色
LIQUID_COLORS = {"red": "#D32F2F", "blue": "#1976D2", "silver": "#0C5245"}

# --- 辅助函数 ---


def celsius_to_fahrenheit(c):
    """摄氏度转华氏度"""
    return c * 9 / 5 + 32


def fahrenheit_to_celsius(f):
    """华氏度转摄氏度"""
    return (f - 32) * 5 / 9


# --- 配置类 ---


class Config:
    """
    生成一个随机的温度计配置方案。
    每次实例化此类，都会得到一套全新的随机属性。
    """

    def __init__(self):
        # 1. 随机选择一个范围和刻度
        self.min_c, self.max_c, self.step_c, self.major_step_c = random.choice(RANGES_C)

        # 2. 随机选择形状
        self.shape = random.choice(SHAPES)

        # 3. 随机选择刻度类型
        # 注意: 根据备注，'on_glass' 形状只能有单刻度
        if self.shape == "on_glass":
            self.scale_type = random.choice(["C", "F"])
        else:
            self.scale_type = random.choice(SCALE_TYPES)

        # 4. 随机选择液体颜色
        self.liquid_color_name = random.choice(list(LIQUID_COLORS.keys()))
        self.liquid_color_hex = LIQUID_COLORS[self.liquid_color_name]

        # 5. 随机选择方向和旋转角度
        self.orientation = random.choice(["vertical", "horizontal"])
        self.angle = random.uniform(-30, 30)

        # 6. 生成随机温度读数
        # 允许读数精度比最小刻度高一位
        precision = 0
        if str(self.step_c).find(".") != -1:
            precision = len(str(self.step_c).split(".")[1]) + 1
        else:
            precision = 1

        self.temp_c = round(random.uniform(self.min_c, self.max_c), precision)
        self.temp_f = celsius_to_fahrenheit(self.temp_c)

        # 7. 计算标注文件所需的 interval
        self._calculate_intervals()

    def _calculate_intervals(self):
        """
        根据随机温度和最小刻度，计算其两侧的刻度值。
        """
        # --- 摄氏度区间 ---
        lower_bound_c = np.floor(self.temp_c / self.step_c) * self.step_c
        upper_bound_c = lower_bound_c + self.step_c
        # 确保格式正确，避免浮点数精度问题
        decimals = str(self.step_c)[::-1].find(".")
        if decimals == -1:  # 如果是整数
            self.celsius_interval = [round(lower_bound_c), round(upper_bound_c)]
        else:
            self.celsius_interval = [
                round(lower_bound_c, decimals),
                round(upper_bound_c, decimals),
            ]

        # --- 华氏度区间 (为简化，直接转换摄氏度区间) ---
        # 注意：在真实世界中，华氏度刻度是独立绘制的，这里做了一个简化
        lower_bound_f = celsius_to_fahrenheit(self.celsius_interval[0])
        upper_bound_f = celsius_to_fahrenheit(self.celsius_interval[1])
        self.fahrenheit_interval = [round(lower_bound_f, 1), round(upper_bound_f, 1)]

    def get_major_tick_positions(self):
        """
        获取大刻度位置，确保在整数倍数上
        """
        # 找到第一个大刻度位置（向上取整到major_step_c的倍数）
        first_major = np.ceil(self.min_c / self.major_step_c) * self.major_step_c

        major_positions = []
        current_pos = first_major
        while current_pos <= self.max_c:
            if current_pos >= self.min_c:  # 确保在范围内
                major_positions.append(current_pos)
            current_pos += self.major_step_c

        return major_positions

    def __str__(self):
        """方便打印配置信息进行调试"""
        return (
            f"--- Thermometer Config ---\n"
            f"Range (C): [{self.min_c}, {self.max_c}], Step: {self.step_c}, Major Step: {self.major_step_c}\n"
            f"Shape: {self.shape}\n"
            f"Scale Type: {self.scale_type}\n"
            f"Liquid Color: {self.liquid_color_name}\n"
            f"Orientation: {self.orientation} at {self.angle:.2f} degrees\n"
            f"Temperature: {self.temp_c:.2f}°C / {self.temp_f:.2f}°F\n"
            f"Celsius Interval: {self.celsius_interval}\n"
            f"Fahrenheit Interval: {self.fahrenheit_interval}\n"
            f"Major Tick Positions: {self.get_major_tick_positions()}\n"
            f"--------------------------"
        )


# --- 测试代码 ---
if __name__ == "__main__":
    random_config = Config()
    print(random_config)

    # 测试 'on_glass' 只能是单刻度
    on_glass_configs = [Config() for _ in range(20) if Config().shape == "on_glass"]
    print("\n--- Testing 'on_glass' constraint ---")
    for cfg in on_glass_configs[:5]:
        print(f"Shape: {cfg.shape}, Scale Type: {cfg.scale_type}")
        assert cfg.scale_type != "C_F"
    print("Test passed: 'on_glass' shape correctly restricts to single scale.")

    # 测试大刻度位置
    print("\n--- Testing major tick positions ---")
    for i in range(3):
        cfg = Config()
        print(f"Range: [{cfg.min_c}, {cfg.max_c}], Major Step: {cfg.major_step_c}")
        print(f"Major Ticks: {cfg.get_major_tick_positions()}")
        print()
