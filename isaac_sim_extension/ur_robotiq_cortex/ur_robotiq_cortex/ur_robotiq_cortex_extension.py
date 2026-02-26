import asyncio
import os
from pathlib import Path

import omni
import omni.ext
import omni.ui as ui
from isaacsim.cortex.framework.cortex_world import CortexWorld
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.examples.interactive.base_sample import BaseSampleUITemplate
from isaacsim.gui.components.ui_utils import btn_builder, get_style, dropdown_builder

from ur_robotiq_cortex.ur_robotiq_cortex import URRobotiqCortex

# Behavior files live in the same package directory as this module.
_EXT_DIR = Path(__file__).resolve().parent


class URRobotiqCortexExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self.example_name = "UR5 HCDT"
        self.category = "Cortex"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "UR5 HCDT",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/cortex_tutorials/tutorial_cortex_4_franka_block_stacking.html",
            "overview": (
                "UR5/UR10 + Robotiq gripper Cortex examples.\n\n"
                "• ROS 2 Follower – subscribes to /joint_states and mirrors the\n"
                "  robot joints in simulation using Curobo.\n"
                "• VLA Control – receives Cartesian actions from a Vision-\n"
                "  Language-Action model over HTTP/ZMQ.\n\n"
                "Environment variables:\n"
                "  UR_ROBOT_TYPE  ur5 (default) | ur10\n"
                "  UR_WS_PATH     path to ur_ws root (auto-detected if unset)"
            ),
        }

        ui_handle = URRobotiqCortexUI(**ui_kwargs)
        ui_handle.sample = URRobotiqCortex(ui_handle.on_diagnostics)

        get_browser_instance().register_example(
            name=self.example_name,
            execute_entrypoint=ui_handle.build_window,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

    def on_shutdown(self):
        get_browser_instance().deregister_example(
            name=self.example_name, category=self.category
        )


class URRobotiqCortexUI(BaseSampleUITemplate):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Behavior paths resolved relative to this module at runtime.
        self.behavior_map = {
            "ROS 2 Follower": str(_EXT_DIR / "ur_ros2_follower_behavior.py"),
            "VLA Control":    str(_EXT_DIR / "ur_vla_behavior.py"),
        }
        self.selected_behavior = "ROS 2 Follower"
        self.loaded = False

    def build_ui(self):
        self.task_ui_elements = {}
        self.build_default_frame()

        with self._controls_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self.task_ui_elements["Selected Behavior"] = dropdown_builder(
                    "Selected Behavior",
                    items=list(self.behavior_map.keys()),
                    on_clicked_fn=self.__on_selected_behavior_changed,
                )

                dict = {
                    "label": "Load World",
                    "type": "button",
                    "text": "Load",
                    "tooltip": "Load World and Task",
                    "on_clicked_fn": self._on_load_world,
                }
                self._buttons["Load World"] = btn_builder(**dict)
                self._buttons["Load World"].enabled = True

                dict = {
                    "label": "Reset",
                    "type": "button",
                    "text": "Reset",
                    "tooltip": "Reset robot and environment",
                    "on_clicked_fn": self._on_reset,
                }
                self._buttons["Reset"] = btn_builder(**dict)
                self._buttons["Reset"].enabled = False

        self.build_extra_frames()

    def build_extra_frames(self):
        extra_stacks = self.get_extra_frames_handle()

        with extra_stacks:
            with ui.CollapsableFrame(
                title="Task Control",
                width=ui.Fraction(0.33),
                height=0,
                visible=True,
                collapsed=False,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            ):
                self.build_task_controls_ui()

            with ui.CollapsableFrame(
                title="Diagnostic",
                width=ui.Fraction(0.33),
                height=0,
                visible=True,
                collapsed=False,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            ):
                self.build_diagnostic_ui()

    def _on_load_world(self):
        self._sample.behavior = self.get_behavior()
        self.loaded = True
        super()._on_load_world()

    def on_diagnostics(self, diagnostic, decision_stack):
        if diagnostic:
            self.diagostic_model.set_value(diagnostic)
        self.state_model.set_value(decision_stack)
        self.diagnostics_panel.visible = bool(diagnostic)

    def get_world(self):
        return CortexWorld.instance()

    def get_behavior(self):
        return self.behavior_map[self.selected_behavior]

    def _on_start_button_event(self):
        asyncio.ensure_future(self.sample.on_event_async())
        self.task_ui_elements["Start"].enabled = False

    def post_reset_button_event(self):
        self.task_ui_elements["Start"].enabled = True

    def post_load_button_event(self):
        self.task_ui_elements["Start"].enabled = True

    def post_clear_button_event(self):
        self.task_ui_elements["Start"].enabled = False

    def __on_selected_behavior_changed(self, selected_index):
        self.selected_behavior = selected_index
        if self.loaded:
            asyncio.ensure_future(self._sample.load_behavior(self.get_behavior()))
            self.on_diagnostics("", "")

    def build_task_controls_ui(self):
        with ui.VStack(spacing=5):
            self.task_ui_elements["Start"] = btn_builder(
                label="Start",
                type="button",
                text="Start",
                tooltip="Start simulation",
                on_clicked_fn=self._on_start_button_event,
            )
            self.task_ui_elements["Start"].enabled = False

    def build_diagnostic_ui(self):
        with ui.VStack(spacing=5):
            ui.Label("Decision Stack", height=20)
            self.state_model = ui.SimpleStringModel()
            ui.StringField(self.state_model, multiline=True, height=120)
            self.diagnostics_panel = ui.VStack(spacing=5)
            with self.diagnostics_panel:
                ui.Label("Diagnostic message", height=20)
                self.diagostic_model = ui.SimpleStringModel()
                ui.StringField(self.diagostic_model, multiline=True, height=200)
