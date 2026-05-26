import cv2
import numpy as np
import math
from typing import Tuple, List
from .config import ProtractorConfig, ProtractorType, ScaleType, StyleType


class ProtractorRenderer:
    """量角器渲染器 - 修复角度体系不一致问题"""

    def __init__(self):
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.5  # 保持字体不变，不随量角器放大
        self.font_thickness = 2

    def render_protractor(self, config: ProtractorConfig) -> np.ndarray:
        """渲染量角器图像"""
        # 创建背景图像
        image = np.full(
            (config.image_height, config.image_width, 3),
            config.color_config.background_color,
            dtype=np.uint8,
        )

        # 绘制量角器主体
        self._draw_protractor_body(image, config)

        # 绘制刻度
        self._draw_scales(image, config)

        # 绘制数字
        self._draw_numbers(image, config)

        # 绘制样式特效
        self._draw_style_effects(image, config)

        # 绘制测量线和箭头
        self._draw_measurement_lines(image, config)

        # 应用整体旋转
        if abs(config.rotation_angle) > 0.1:
            image = self._rotate_image(image, config.rotation_angle, config.center)

        return image

    def _draw_protractor_body(self, image: np.ndarray, config: ProtractorConfig):
        center = config.center
        radius = config.radius
        color = config.color_config.protractor_color

        if config.protractor_type == ProtractorType.HALF_CIRCLE:
            # 半圆主体
            cv2.ellipse(image, center, (radius, radius), 0, 180, 360, color, 4)
            cv2.line(
                image,
                (center[0] - radius, center[1]),
                (center[0] + radius, center[1]),
                color,
                4,
            )
        else:
            cv2.circle(image, center, radius, color, 4)

    def _draw_scales(self, image: np.ndarray, config: ProtractorConfig):
        angle_range = (
            180 if config.protractor_type == ProtractorType.HALF_CIRCLE else 360
        )
        for angle in np.arange(0, angle_range + 1, config.scale_config.min_scale):
            self._draw_single_scale(image, config, angle)

    def _draw_single_scale(
        self, image: np.ndarray, config: ProtractorConfig, angle: float
    ):
        center = config.center
        radius = config.radius
        sc = config.scale_config
        color = config.color_config.scale_color

        # 确定刻度类型
        if angle % sc.number_scale == 0:
            length, thickness = sc.number_scale_length, int(sc.number_scale_width)
        elif angle % sc.major_scale == 0:
            length, thickness = sc.major_scale_length, int(sc.major_scale_width)
        elif angle % sc.min_scale == 0:
            length, thickness = sc.min_scale_length, int(sc.min_scale_width)
        else:
            return

        actual_angles = self._get_scale_angles(
            angle, config.scale_type, config.protractor_type
        )
        for actual_angle in actual_angles:
            rad = math.radians(actual_angle)
            # OpenCV angle starts from 3 o'clock and goes counter-clockwise.
            # Y-axis is inverted in image coordinates.
            outer_r = radius - 5
            inner_r = radius - length - 5
            start_point = (
                int(center[0] + inner_r * math.cos(rad)),
                int(center[1] - inner_r * math.sin(rad)),
            )
            end_point = (
                int(center[0] + outer_r * math.cos(rad)),
                int(center[1] - outer_r * math.sin(rad)),
            )
            cv2.line(image, start_point, end_point, color, max(1, thickness))

    def _get_scale_angles(
        self, angle: float, scale_type: ScaleType, protractor_type: ProtractorType
    ) -> List[float]:
        """
        Converts a display angle into the corresponding mathematical angle(s) for drawing.
        Mathematical angle system: 0° is on the right (3 o'clock), positive is counter-clockwise.
        """
        angles = []
        if protractor_type == ProtractorType.HALF_CIRCLE:
            if scale_type == ScaleType.CLOCKWISE:
                # A clockwise reading 'a' is at mathematical angle '180 - a'.
                angles.append(180.0 - angle)
            elif scale_type == ScaleType.COUNTERCLOCKWISE:
                # The reading angle is the mathematical angle.
                angles.append(angle)
            elif scale_type == ScaleType.DUAL:
                # For a dual scale, draw both counter-clockwise and clockwise ticks.
                angles.append(angle)
                angles.append(180.0 - angle)
        else:  # FULL_CIRCLE
            if scale_type == ScaleType.CLOCKWISE:
                angles.append((360.0 - angle) % 360.0)
            elif scale_type == ScaleType.COUNTERCLOCKWISE:
                angles.append(angle)
            elif scale_type == ScaleType.DUAL:
                angles.append(angle)
                angles.append((360.0 - angle) % 360.0)
        return angles

    def _draw_numbers(self, image: np.ndarray, config: ProtractorConfig):
        angle_range = (
            180 if config.protractor_type == ProtractorType.HALF_CIRCLE else 360
        )
        for angle in range(0, angle_range + 1, int(config.scale_config.number_scale)):
            # For dual scales, we need to handle number placement carefully to avoid overlap.
            # Here we draw numbers for both scales.
            if (
                config.scale_type == ScaleType.DUAL
                and config.protractor_type == ProtractorType.HALF_CIRCLE
            ):
                # Draw counter-clockwise number
                math_angle_ccw = self._get_mathematical_angle_for_measurement(
                    angle, ScaleType.COUNTERCLOCKWISE, config.protractor_type
                )
                self._draw_single_number(
                    image, config, angle, math_angle_ccw, is_outer=True
                )
                # Draw clockwise number (avoid double-drawing 90)
                if angle != 90:
                    math_angle_cw = self._get_mathematical_angle_for_measurement(
                        angle, ScaleType.CLOCKWISE, config.protractor_type
                    )
                    self._draw_single_number(
                        image, config, angle, math_angle_cw, is_outer=False
                    )
            else:
                actual_angles = self._get_scale_angles(
                    angle, config.scale_type, config.protractor_type
                )
                for actual_angle in actual_angles:
                    self._draw_single_number(image, config, angle, actual_angle)

    def _draw_single_number(
        self, image, config, display_angle, actual_angle, is_outer=True
    ):
        center = config.center
        radius = config.radius
        color = config.color_config.number_color
        # Adjust text radius for inner/outer scales if needed
        offset = 15 if is_outer else 45
        text_radius = radius - config.scale_config.number_scale_length - offset

        rad = math.radians(actual_angle)
        x = center[0] + text_radius * math.cos(rad)
        y = center[1] - text_radius * math.sin(rad)

        text = str(display_angle)
        (text_width, text_height), _ = cv2.getTextSize(
            text, self.font, self.font_scale, self.font_thickness
        )

        # Center the text on its calculated position
        x_pos = int(x - text_width / 2)
        y_pos = int(y + text_height / 2)

        cv2.putText(
            image,
            text,
            (x_pos, y_pos),
            self.font,
            self.font_scale,
            color,
            self.font_thickness,
        )

    def _draw_style_effects(self, image: np.ndarray, config: ProtractorConfig):
        if config.style_type == StyleType.RADIAL_LINES:
            self._draw_radial_lines(image, config)
        elif config.style_type == StyleType.HOLLOW_CENTER:
            self._draw_hollow_center(image, config)
        elif config.style_type == StyleType.CENTER_MARKS:
            self._draw_center_marks(image, config)

    def _draw_radial_lines(self, image, config):
        center, radius = config.center, config.radius
        color = config.color_config.scale_color
        step = int(config.scale_config.number_scale)
        angle_range = (
            180 if config.protractor_type == ProtractorType.HALF_CIRCLE else 360
        )
        for angle in range(0, angle_range + 1, step):
            for actual_angle in self._get_scale_angles(
                angle, config.scale_type, config.protractor_type
            ):
                rad = math.radians(actual_angle)
                end = (
                    int(center[0] + (radius - 5) * math.cos(rad)),
                    int(center[1] - (radius - 5) * math.sin(rad)),
                )
                cv2.line(image, center, end, color, 1)

    def _draw_hollow_center(self, image, config):
        center, radius = config.center, config.radius
        r = radius // 4
        if config.protractor_type == ProtractorType.HALF_CIRCLE:
            cv2.ellipse(
                image,
                center,
                (r, r),
                0,
                180,
                360,
                config.color_config.background_color,
                -1,
            )
            cv2.ellipse(
                image,
                center,
                (r, r),
                0,
                180,
                360,
                config.color_config.protractor_color,
                2,
            )
        else:
            cv2.circle(image, center, r, config.color_config.background_color, -1)
            cv2.circle(image, center, r, config.color_config.protractor_color, 2)

    def _draw_center_marks(self, image, config):
        center = config.center
        color = config.color_config.scale_color
        for angle in range(0, 360, 30):
            if config.protractor_type == ProtractorType.HALF_CIRCLE and (
                angle > 180 and angle < 360
            ):
                continue
            rad = math.radians(angle)
            start = (
                int(center[0] + 8 * math.cos(rad)),
                int(center[1] - 8 * math.sin(rad)),
            )
            end = (
                int(center[0] + 23 * math.cos(rad)),
                int(center[1] - 23 * math.sin(rad)),
            )
            cv2.line(image, start, end, color, 2)

    def _get_mathematical_angle_for_measurement(
        self,
        reading_angle: float,
        scale_type: ScaleType,
        protractor_type: ProtractorType,
    ) -> float:
        """
        Converts a measurement reading angle to a mathematical angle for drawing.
        For DUAL scale, we assume the primary (counter-clockwise) scale is used.
        """
        if protractor_type == ProtractorType.HALF_CIRCLE:
            if scale_type == ScaleType.CLOCKWISE:
                return 180.0 - reading_angle
            else:  # COUNTERCLOCKWISE or DUAL assumes the primary CCW scale
                return reading_angle
        else:  # FULL_CIRCLE
            if scale_type == ScaleType.CLOCKWISE:
                return (360.0 - reading_angle) % 360.0
            else:  # COUNTERCLOCKWISE or DUAL
                return reading_angle

    def _draw_measurement_lines(self, image, config):
        center, radius = config.center, config.radius
        line_color = config.color_config.line_color
        line_len = int(radius * 1.05)  # Extend lines slightly beyond the radius

        # Convert reading angles to mathematical angles to match the rendered scale
        math_angle1 = self._get_mathematical_angle_for_measurement(
            config.angle1, config.scale_type, config.protractor_type
        )
        math_angle2 = self._get_mathematical_angle_for_measurement(
            config.angle2, config.scale_type, config.protractor_type
        )

        for rad in (math.radians(math_angle1), math.radians(math_angle2)):
            end = (
                int(center[0] + line_len * math.cos(rad)),
                int(center[1] - line_len * math.sin(rad)),
            )
            cv2.line(image, center, end, line_color, 3)

        self._draw_angle_arc(image, config, line_len // 3, math_angle1, math_angle2)

    def _draw_angle_arc(self, image, config, arc_radius, a1, a2):
        center = config.center
        arrow_color = config.color_config.arrow_color
        # cv2.ellipse expects angles in degrees, clockwise from the 3 o'clock position
        # We negate our standard mathematical angles to convert them to OpenCV's format
        start_angle, end_angle = min(a1, a2), max(a1, a2)
        cv2.ellipse(
            image,
            center,
            (arc_radius, arc_radius),
            0,
            -end_angle,
            -start_angle,
            arrow_color,
            2,
        )

    def _rotate_image(
        self, image: np.ndarray, angle: float, center: Tuple[int, int]
    ) -> np.ndarray:
        """Rotates an image around a center point."""
        rows, cols = image.shape[:2]
        # cv2.getRotationMatrix2D expects a positive angle for counter-clockwise rotation
        M = cv2.getRotationMatrix2D(center, angle, 1.0)
        # Use a neutral border color, like a light gray, during rotation
        return cv2.warpAffine(image, M, (cols, rows), borderValue=(240, 240, 240))

    def save_image(self, image, filepath):
        cv2.imwrite(filepath, image)

    def get_image_info(self, config: ProtractorConfig) -> dict:
        return {
            "image_width": config.image_width,
            "image_height": config.image_height,
            "center": config.center,
            "radius": config.radius,
            "rotation_angle": config.rotation_angle,
        }
