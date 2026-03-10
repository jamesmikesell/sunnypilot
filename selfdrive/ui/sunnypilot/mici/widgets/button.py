import pyray as rl

from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigMultiParamToggle

POLYGON_FILL_COLOR = rl.Color(255, 255, 255, int(255 * 0.90))
POLYGON_OUTLINE_COLOR = rl.Color(0xAA, 0xAA, 0xAA, 255)
POLYGON_DISABLED_FILL_COLOR = rl.Color(255, 255, 255, int(255 * 0.35))
POLYGON_DISABLED_OUTLINE_COLOR = rl.Color(0x66, 0x66, 0x66, 255)


class BigPolygonMultiParamToggle(BigMultiParamToggle):
  INDICATOR_RADIUS = 26.0
  INDICATOR_OUTLINE_THICKNESS = 4.0
  INDICATOR_X_PADDING = 42.0
  BASE_SIDES = 3

  def _draw_content(self, btn_y: float):
    BigButton._draw_content(self, btn_y)

    option_idx = self._options.index(self.value)
    sides = option_idx + self.BASE_SIDES

    center = rl.Vector2(
      self._rect.x + self._rect.width - self.INDICATOR_X_PADDING,
      btn_y + self._rect.height / 2,
    )

    fill_color = POLYGON_FILL_COLOR if self.enabled else POLYGON_DISABLED_FILL_COLOR
    outline_color = POLYGON_OUTLINE_COLOR if self.enabled else POLYGON_DISABLED_OUTLINE_COLOR

    rl.draw_poly(center, sides, self.INDICATOR_RADIUS, -90.0, fill_color)
    rl.draw_poly_lines_ex(center, sides, self.INDICATOR_RADIUS, -90.0,
                          self.INDICATOR_OUTLINE_THICKNESS, outline_color)
