import pyray as rl
from dataclasses import dataclass

from cereal import car
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.lib.multilang import FontWeight

# Gear indicator replaces driver monitor circle (same position/size as DM widget)
BG_SIZE = 120
GEAR_COLOR = rl.Color(26, 242, 66, 255)  # same green as engaged DM
BG_COLOR = rl.Color(0, 0, 0, 166)


@dataclass
class GearData:
  gear_text: str = "-"
  gear_shifter: str = "unknown"
  faceOrientation: tuple = (0.0, 0.0, 0.0)
  facePosition: tuple = (0.0, 0.0)
  faceOrientationStd: tuple = (0.0, 0.0, 0.0)
  valid: bool = True


class DriverStateRenderer(Widget):
  """Renders current transmission gear in the driver-monitor circle slot."""

  def __init__(self, inset: bool = False, lines: bool = False):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, BG_SIZE, BG_SIZE))
    self._inset = inset
    self._lines = lines
    self._is_rhd = ui_state.is_rhd
    self._should_draw = True
    self._force_active = False
    self._gear_data = GearData()
    self._font = gui_app.font(FontWeight.DISPLAY)

  @property
  def is_rhd(self) -> bool:
    return self._is_rhd

  def set_should_draw(self, should_draw: bool):
    self._should_draw = should_draw

  def set_force_active(self, force_active: bool):
    self._force_active = force_active

  def get_driver_data(self) -> GearData:
    return self._gear_data

  def load_icons(self):
    pass

  def _gear_text(self, current_gear: int, gear_shifter: car.CarState.GearShifter) -> str:
    if current_gear == 13:
      return "R"
    if 1 <= current_gear <= 10:
      return str(current_gear)
    if gear_shifter == car.CarState.GearShifter.reverse:
      return "R"
    if gear_shifter == car.CarState.GearShifter.park:
      return "P"
    if gear_shifter == car.CarState.GearShifter.neutral:
      return "N"
    return "-"

  def _update_state(self):
    if not ui_state.started:
      return

    sm = ui_state.sm
    if not sm.updated['carState']:
      return

    cs = sm['carState']
    self._gear_data = GearData(
      gear_text=self._gear_text(cs.currentGear, cs.gearShifter),
      gear_shifter=str(cs.gearShifter),
    )

  def _render(self, _rect: rl.Rectangle):
    if not self._should_draw and not self._force_active:
      return

    bg_rect = rl.Rectangle(self._rect.x, self._rect.y, BG_SIZE, BG_SIZE)
    rl.draw_circle(int(bg_rect.x + bg_rect.width / 2), int(bg_rect.y + bg_rect.height / 2),
                   int(bg_rect.width / 2), BG_COLOR)

    gear_text = self._gear_data.gear_text
    font_size = 64 if len(gear_text) == 1 else 48
    text_size = measure_text_cached(self._font, gear_text, font_size, 1.0)
    text_pos = rl.Vector2(
      bg_rect.x + (bg_rect.width - text_size.x) / 2,
      bg_rect.y + (bg_rect.height - text_size.y) / 2 - 4,
    )
    rl.draw_text_ex(self._font, gear_text, text_pos, font_size, 1.0, GEAR_COLOR)

  @property
  def should_draw(self) -> bool:
    return (self._should_draw and
            ui_state.sm['selfdriveState'].alertSize == car.SelfdriveState.AlertSize.none)
