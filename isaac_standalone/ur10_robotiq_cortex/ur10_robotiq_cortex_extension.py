# SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import asyncio
import os

import omni
import omni.ext
import omni.ui as ui
from isaacsim.cortex.framework.cortex_world import CortexWorld
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.examples.interactive.base_sample import BaseSampleUITemplate
from isaacsim.examples.interactive.ur10_robotiq_cortex.ur10_robotiq_cortex import UR10RobotiqCortex
from isaacsim.gui.components.ui_utils import btn_builder, cb_builder, dropdown_builder, get_style, str_builder
from isaacsim.examples.interactive.ur10_robotiq_cortex import mr_overlay
class UR10RobotiqCortexExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self.example_name = "UR10 Robotiq Cortex Examples"
        self.category = "Cortex"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "UR10 Robotiq Cortex Examples",
            "doc_link": "https://docs.isaacsim.omniverse.nvidia.com/latest/cortex_tutorials/tutorial_cortex_4_franka_block_stacking.html#isaac-sim-app-tutorial-cortex-4-franka-block-stacking",
            "overview": "This Example shows how to use Cortex with a UR10 robot equipped with a Robotiq 2F-140 gripper. Open 'Link to Docs' to see more detailed instructions on how to run this example. \n\nPress the 'Open in IDE' button to view the source code.",
        }

        ui_handle = UR10RobotiqCortexUI(**ui_kwargs)

        ui_handle.sample = UR10RobotiqCortex(ui_handle.on_diagnostics)

        get_browser_instance().register_example(
            name=self.example_name,
            execute_entrypoint=ui_handle.build_window,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

        return

    def on_shutdown(self):
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
        return


class UR10RobotiqCortexUI(BaseSampleUITemplate):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Use complete absolute paths
        simple_behavior_path = "/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/exts/isaacsim.examples.interactive/isaacsim/examples/interactive/ur10_robotiq_cortex/simple_ur10_behavior.py"
        block_stacking_path = "/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/exts/isaacsim.examples.interactive/isaacsim/examples/interactive/ur10_robotiq_cortex/ur10_block_stacking_behavior.py"
        vla_behavior_path = "/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/exts/isaacsim.examples.interactive/isaacsim/examples/interactive/ur10_robotiq_cortex/ur10_vla_behavior.py"
        ros2_follower_path = "/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/exts/isaacsim.examples.interactive/isaacsim/examples/interactive/ur10_robotiq_cortex/ur10_ros2_follower_behavior.py"
        
        # example starter parameters - using full absolute paths
        self.behavior_map = {
            "Simple Behavior": simple_behavior_path,
            "Block Stacking": block_stacking_path,
            "VLA Control": vla_behavior_path,
            "ROS 2 Follower": ros2_follower_path,
        }
        self.selected_behavior = "ROS 2 Follower"  # Default to ROS 2 Follower for convenience
        self.loaded = False

    def build_ui(self):
        # overwriting the baseSample's default frame
        self.task_ui_elements = {}
        self.build_default_frame()

        # modification to the control frame
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
        return

    def _on_mr_overlay_button_event(self):
        """UI callback to generate a mixed-reality overlay image.

        This uses the helper data exposed by UR10RobotiqCortex.setup_scene:
        _mr_cube_layout and _mr_input_image. If either is missing,
        the button becomes a no-op.
        """

        sample = getattr(self, "sample", None)
        if sample is None:
            # Sample not yet constructed.
            return

        cube_layout = getattr(sample, "_mr_cube_layout", None)
        image_path = getattr(sample, "_mr_input_image", None)

        if not cube_layout or not image_path:
            # MR data not available yet (e.g., setup_scene not run).
            return

        try:
            output_path = mr_overlay.generate_mr_overlay(image_path, cube_layout)
            print(f"[UR10 MR] Mixed-reality overlay generated at: {output_path}")
        except Exception as exc:
            # Keep UI robust: log the error but do not raise.
            print(f"[UR10 MR] Failed to generate MR overlay: {exc}")

    def post_reset_button_event(self):
        self.task_ui_elements["Start"].enabled = True
        if "MR Overlay" in self.task_ui_elements:
            self.task_ui_elements["MR Overlay"].enabled = False
        return

    def post_load_button_event(self):
        self.task_ui_elements["Start"].enabled = True
        # World and behavior are loaded at this point; MR overlay has access
        # to cube layout and reference image path.
        if "MR Overlay" in self.task_ui_elements:
            self.task_ui_elements["MR Overlay"].enabled = True
        return

    def post_clear_button_event(self):
        self.task_ui_elements["Start"].enabled = False
        if "MR Overlay" in self.task_ui_elements:
            self.task_ui_elements["MR Overlay"].enabled = False
        return

    def __on_selected_behavior_changed(self, selected_index):
        self.selected_behavior = selected_index
        if self.loaded:
            asyncio.ensure_future(self._sample.load_behavior(self.get_behavior()))
            self.on_diagnostics("", "")

    def build_task_controls_ui(self):
        with ui.VStack(spacing=5):
            dict = {
                "label": "Start",
                "type": "button",
                "text": "Start",
                "tooltip": "Start",
                "on_clicked_fn": self._on_start_button_event,
            }
            self.task_ui_elements["Start"] = btn_builder(**dict)
            self.task_ui_elements["Start"].enabled = False

            # Button to generate a mixed-reality overlay image.
            # This is optional and does not affect the main control flow.
            dict = {
                "label": "Generate MR Overlay",
                "type": "button",
                "text": "MR Overlay",
                "tooltip": "Generate mixed-reality overlay image using the current cube layout.",
                "on_clicked_fn": self._on_mr_overlay_button_event,
            }
            self.task_ui_elements["MR Overlay"] = btn_builder(**dict)
            # Start disabled; will be enabled after world load.
            self.task_ui_elements["MR Overlay"].enabled = False

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
