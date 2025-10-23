#pragma once
#include <std_msgs/msg/color_rgba.hpp>

namespace o3d_slam {

class Color {
 public:
  Color();
  Color(double red, double green, double blue);
  Color(double red, double green, double blue, double alpha);

  Color operator*(double scalar) const;
  static Color White() { return Color(1.0, 1.0, 1.0); }
  static Color Black() { return Color(0.0, 0.0, 0.0); }
  static Color Gray() { return Color(0.5, 0.5, 0.5); }
  static Color Red() { return Color(1.0, 0.0, 0.0); }
  static Color Green() { return Color(0.0, 1.0, 0.0); }
  static Color Blue() { return Color(0.0, 0.0, 1.0); }
  static Color Yellow() { return Color(1.0, 1.0, 0.0); }
  static Color Orange() { return Color(1.0, 0.5, 0.0); }
  static Color Purple() { return Color(0.5, 0.0, 1.0); }
  static Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
  static Color Teal() { return Color(0.0, 1.0, 1.0); }
  static Color Pink() { return Color(1.0, 0.0, 0.5); }
  static Color Magenta() { return Color(0.78, 0.0, 0.9); }
  static const int numColors_ = 13;
  static Color getColor(int colorCode);

  std_msgs::msg::ColorRGBA msg_;
};

Color operator*(double scalar, const Color& c);

} // namespace o3d_slam
