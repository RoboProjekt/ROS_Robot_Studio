"""Kinematics, teaching and sequential preview integrated beside the existing robot viewer."""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import yaml
from PySide6.QtCore import QObject, Qt, QTimer
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QScrollArea,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from ros_robot_studio.gui.motion_dialogs import (
    PickPlaceWizard,
    PoseEditor,
    PositionDialog,
    StepDialog,
    number,
)
from ros_robot_studio.gui.widgets import button, label
from ros_robot_studio.gui.workers import TaskRunner
from ros_robot_studio.kinematics.engine import UniversalKinematicsEngine
from ros_robot_studio.kinematics.solvers import KinematicsSolverManager
from ros_robot_studio.motion.backend import OfflinePreviewBackend
from ros_robot_studio.motion.positions import PositionManager
from ros_robot_studio.motion.program import COLLISION_NOTICE, TaskCompiler
from ros_robot_studio.motion.schema import KinematicSettings, MotionTask, SavedPosition
from ros_robot_studio.projects.store import atomic_text
from ros_robot_studio.robots.urdf import pose_rpy


class MotionWorkspace(QObject):
    def __init__(self, main):
        super().__init__(main)
        self.main, self.robot = main, main.robot_panel
        self.runner = TaskRunner(self)
        self.runner.failed.connect(self.error)
        self.runner.busy_changed.connect(self.busy_changed)
        self.runner.progress.connect(lambda text: self.task_status.setText(text))
        self.engine, self.chain, self.result, self.plan = None, None, None, None
        self.positions = PositionManager()
        self.tasks = []
        self.config = KinematicSettings()
        self.project_id = None
        self.force_reload = True
        self.generation = 0
        self.cancel_event = threading.Event()
        self.loading = False
        self.backend = OfflinePreviewBackend()
        self.wizard = None
        self.was_installed = False
        self.previewing_ik = False
        self.build_kinematics()
        self.build_programs()
        self.robot.detail_tabs.addTab(self.kin_panel, "Kinematics & Motion")
        self.robot.detail_tabs.addTab(self.program_panel, "Positions & Tasks")
        self.robot.robot_changed.connect(self.bind_robot)
        self.robot.changed.connect(self.robot_edited)
        self.robot.end_effector.currentTextChanged.connect(self.external_frame_changed)
        self.timer = QTimer(self)
        self.timer.setInterval(33)
        self.timer.timeout.connect(self.tick)
        self.timer.start()
        self.last_tick = time.monotonic()

    def build_kinematics(self):
        self.kin_panel = QScrollArea()
        self.kin_panel.setWidgetResizable(True)
        content = QWidget()
        self.kin_panel.setWidget(content)
        layout = QVBoxLayout(content)
        layout.setContentsMargins(10, 12, 10, 12)
        layout.addWidget(label("Universal kinematics", "section"))
        self.ready = label("Analyzing robot…", "subtitle", True)
        layout.addWidget(self.ready)
        self.install_button = button(
            "Install Universal Kinematics Support…", self.install_support, True
        )
        layout.addWidget(self.install_button)
        tabs = QTabWidget()
        layout.addWidget(tabs, 1)
        cart = QWidget()
        body = QVBoxLayout(cart)
        self.current_pose = label("No chain", "subtitle", True)
        body.addWidget(self.current_pose)
        self.pose_holder = QVBoxLayout()
        body.addLayout(self.pose_holder)
        self.pose_editor = None
        row = QHBoxLayout()
        self.calculate = button("Calculate IK", self.calculate_ik, True)
        self.cancel = button("Cancel", self.cancel_work)
        row.addWidget(self.calculate)
        row.addWidget(self.cancel)
        body.addLayout(row)
        row = QHBoxLayout()
        self.preview = button("Preview Pose", self.preview_ik)
        self.apply = button("Apply to Offline Model", self.apply_ik)
        row.addWidget(self.preview)
        row.addWidget(self.apply)
        body.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(button("Save Position", self.save_cartesian_target))
        row.addWidget(button("Reset target / preview", self.reset_target))
        body.addLayout(row)
        self.ik_output = QPlainTextEdit()
        self.ik_output.setReadOnly(True)
        self.ik_output.setMinimumHeight(95)
        self.ik_output.setMaximumBlockCount(150)
        body.addWidget(self.ik_output, 1)
        self.gripper_form = QFormLayout()
        body.addWidget(label("Virtual gripper · URDF units", "section"))
        body.addLayout(self.gripper_form)
        self.gripper_spins = {}
        self.apply_gripper_button = button("Apply virtual gripper", self.apply_gripper)
        body.addWidget(self.apply_gripper_button)
        tabs.addTab(cart, "Cartesian control")
        setup = QWidget()
        setup_body = QVBoxLayout(setup)
        form = QFormLayout()
        self.base = QComboBox()
        self.end = QComboBox()
        self.gripper = QComboBox()
        self.solver = QComboBox()
        self.solver.addItem("Automatic · SciPy bounded IK", "auto")
        for descriptor in KinematicsSolverManager().catalog():
            self.solver.addItem(descriptor.name, descriptor.id)
            self.solver.setItemData(
                self.solver.count() - 1,
                "Available"
                if descriptor.implemented and descriptor.installed
                else "Not implemented"
                if not descriptor.implemented
                else "Dependency missing",
                Qt.ItemDataRole.ToolTipRole,
            )
        form.addRow("Base frame", self.base)
        form.addRow("End-effector frame", self.end)
        form.addRow("Gripper joint", self.gripper)
        form.addRow("Offline solver", self.solver)
        setup_body.addLayout(form)
        self.solver_info = label("", "subtitle", True)
        setup_body.addWidget(self.solver_info)
        setup_body.addWidget(label("Active arm joints (gripper excluded)", "subtitle"))
        self.active = QListWidget()
        self.active.setMaximumHeight(140)
        setup_body.addWidget(self.active)
        self.confirm = button("Confirm chain & run FK / IK checks", self.confirm_chain, True)
        setup_body.addWidget(self.confirm)
        tolerances = QFormLayout()
        self.position_tolerance = number(0.1, 0.001, 10, 4, " mm")
        self.angle_tolerance = number(math.degrees(0.001), 0.001, 10, 4, " °")
        self.starts = QSpinBox()
        self.starts.setRange(1, 16)
        self.starts.setValue(4)
        tolerances.addRow("Position tolerance", self.position_tolerance)
        tolerances.addRow("Angle tolerance", self.angle_tolerance)
        tolerances.addRow("Deterministic starts", self.starts)
        setup_body.addLayout(tolerances)
        self.chain_info = label("", "subtitle", True)
        setup_body.addWidget(self.chain_info)
        setup_body.addStretch()
        tabs.addTab(setup, "Chain setup")
        for combo in (self.base, self.end, self.gripper):
            combo.currentIndexChanged.connect(self.chain_selection_changed)
        self.active.itemChanged.connect(self.config_edited)
        self.solver.currentIndexChanged.connect(self.solver_selected)
        for spin in (self.position_tolerance, self.angle_tolerance, self.starts):
            spin.valueChanged.connect(self.options_edited)
        layout.addWidget(label(COLLISION_NOTICE, "warning", True))

    def build_programs(self):
        self.program_panel = QWidget()
        layout = QVBoxLayout(self.program_panel)
        layout.setContentsMargins(10, 12, 10, 12)
        self.program_tabs = QTabWidget()
        layout.addWidget(self.program_tabs, 1)
        teach = QWidget()
        body = QVBoxLayout(teach)
        row = QHBoxLayout()
        row.addWidget(button("Capture joints", lambda: self.add_position("joint")))
        row.addWidget(button("Capture Cartesian", lambda: self.add_position("cartesian")))
        body.addLayout(row)
        self.position_table = QTableWidget(0, 3)
        self.position_table.setHorizontalHeaderLabels(["POSITION", "TYPE", "BINDING / FRAME"])
        self.configure_table(self.position_table)
        body.addWidget(self.position_table, 1)
        self.position_table.cellDoubleClicked.connect(lambda *_: self.edit_position())
        row = QHBoxLayout()
        row.addWidget(button("Edit", self.edit_position))
        row.addWidget(button("Duplicate", self.duplicate_position))
        row.addWidget(button("Delete", self.delete_position))
        body.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(button("Recall / apply offline", lambda: self.recall_position(True)))
        row.addWidget(button("Check reachability", lambda: self.recall_position(False)))
        body.addLayout(row)
        self.position_status = label(
            "Joint targets retain their robot binding. Cartesian targets are solved again.",
            "subtitle",
            True,
        )
        body.addWidget(self.position_status)
        self.program_tabs.addTab(teach, "Teach-in positions")
        editor = QWidget()
        body = QVBoxLayout(editor)
        row = QHBoxLayout()
        self.task_choice = QComboBox()
        self.task_choice.currentIndexChanged.connect(self.task_selected)
        row.addWidget(self.task_choice, 1)
        row.addWidget(button("New", self.new_task))
        row.addWidget(button("Rename", self.rename_task))
        row.addWidget(button("Delete", self.delete_task))
        body.addLayout(row)
        self.wizard_button = button("Pick-and-Place Wizard…", self.open_wizard, True)
        body.addWidget(self.wizard_button)
        self.steps = QTableWidget(0, 4)
        self.steps.setHorizontalHeaderLabels(["STEP", "ACTION", "TARGET", "STATUS"])
        self.configure_table(self.steps)
        self.steps.cellDoubleClicked.connect(lambda *_: self.edit_step())
        body.addWidget(self.steps, 1)
        row = QHBoxLayout()
        for title, callback in [
            ("Add", self.add_step),
            ("Edit", self.edit_step),
            ("Remove", self.remove_step),
            ("↑", lambda: self.move_step(-1)),
            ("↓", lambda: self.move_step(1)),
        ]:
            row.addWidget(button(title, callback))
        body.addLayout(row)
        settings = QHBoxLayout()
        settings.addWidget(QLabel("Speed"))
        self.speed = QSpinBox()
        self.speed.setRange(1, 100)
        self.speed.setSuffix(" %")
        self.speed.setValue(25)
        self.speed.valueChanged.connect(self.speed_changed)
        settings.addWidget(self.speed)
        movement = QComboBox()
        movement.addItems(["Joint space · cubic", "Cartesian straight path · not implemented"])
        movement.model().item(1).setEnabled(False)
        settings.addWidget(movement, 1)
        body.addLayout(settings)
        row = QHBoxLayout()
        self.validate_task_button = button("Validate all", lambda: self.compile_task(False))
        self.single_step_button = button("Compute selected step", self.compile_selected)
        row.addWidget(self.validate_task_button)
        row.addWidget(self.single_step_button)
        body.addLayout(row)
        row = QHBoxLayout()
        self.backend_choice = QComboBox()
        self.backend_choice.addItems(["Offline Preview", "ROS Simulation"])
        self.backend_choice.currentIndexChanged.connect(self.choose_backend)
        body.addWidget(self.backend_choice)
        self.play_button = button("▶ Preview", lambda: self.compile_task(True), True)
        self.pause_button = button("Ⅱ Pause / resume", self.pause)
        self.stop_button = button("■ Stop", self.stop)
        self.reset_button = button("Reset", self.reset)
        for widget in (self.play_button, self.pause_button, self.stop_button, self.reset_button):
            row.addWidget(widget)
        body.addLayout(row)
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        body.addWidget(self.progress)
        self.task_status = label("No task validated", "subtitle", True)
        body.addWidget(self.task_status)
        self.program_tabs.addTab(editor, "Task editor")
        row = QHBoxLayout()
        row.addWidget(button("Export positions / tasks…", self.export_library))
        row.addWidget(button("Import library…", self.import_library))
        layout.addLayout(row)
        layout.addWidget(
            label(
                COLLISION_NOTICE
                + "\nVirtual cube attachment only; no forces, contact or stability simulation.",
                "warning",
                True,
            )
        )

    @staticmethod
    def configure_table(table):
        from PySide6.QtWidgets import QHeaderView

        table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)

    def prepare_project_reload(self):
        self.force_reload = True

    def bind_robot(self, robot, model):
        self.cancel_event.set()
        self.stop()
        self.generation += 1
        generation = self.generation
        self.result, self.plan = None, None
        self.previewing_ik = False
        if self.force_reload or self.project_id != self.main.project.id:
            self.positions = PositionManager(self.main.project.positions)
            self.tasks = [t.model_copy(deep=True) for t in self.main.project.tasks]
            config = self.main.project.kinematics.model_copy(deep=True)
        else:
            config = self.config.model_copy(deep=True)
        self.force_reload = False
        self.project_id = self.main.project.id
        current = dict(self.robot.joint_controls.values)
        grippers = []
        if robot.model_id:
            grippers = list(
                self.main.model_manager.spec(robot.model_id, robot.source_revision).gripper_joints
            )
        selected_tool = self.robot.end_effector.currentText()
        self.kinematics_ready = False
        self.ready.setText("Analyzing robot… · offline only")
        self.engine = self.chain = None
        self.robot.view.set_virtual_object(None)
        self.busy_changed(True)

        def work(_):
            engine = UniversalKinematicsEngine(model)
            if (
                config.model_signature != engine.signature
                or not config.base_link
                or not config.end_effector
            ):
                guessed = grippers or [
                    j.name
                    for j in model.joints
                    if j.movable
                    and not j.mimic
                    and any(word in j.name.casefold() for word in ("gripper", "finger", "jaw"))
                ]
                config.model_signature = engine.signature
                config.base_link = model.root
                config.end_effector = (
                    engine.end_effector_candidates[0]
                    if not robot.end_effector and len(engine.end_effector_candidates) == 1
                    else selected_tool
                    if selected_tool in model.links
                    else engine.end_effector_candidates[0]
                )
                config.gripper_joints = guessed
                config.active_joints = []
                config.confirmed = bool(grippers) or (
                    len(engine.end_effector_candidates) == 1 and not guessed
                )
                config.mode = "position_only"
            chain = engine.create_chain(
                config.base_link,
                config.end_effector,
                config.active_joints if config.active_joints else None,
                config.gripper_joints,
            )
            config.active_joints = list(chain.active_joints)
            analysis = chain.analyze()
            installed = KinematicsSolverManager().catalog()[0].installed
            test = (
                KinematicsSolverManager().solve(
                    chain,
                    chain.forward_kinematics(current),
                    current,
                    solver=config.solver,
                    options=config.options,
                )
                if installed and chain.active_joints
                else None
            )
            return engine, chain, config, analysis, test

        def begin():
            if generation != self.generation:
                return
            if self.runner.busy:
                QTimer.singleShot(50, begin)
                return
            self.runner.run(work, lambda result: self.bound(result, generation))

        begin()

    def bound(self, result, generation):
        if generation != self.generation:
            return
        self.engine, self.chain, self.config, analysis, test = result
        self.robot.end_effector.blockSignals(True)
        self.robot.end_effector.setCurrentText(self.config.end_effector)
        self.robot.end_effector.blockSignals(False)
        self.robot.update_pose()
        self.main.state_manager.set_frames(self.config.base_link, self.config.end_effector)
        self.was_installed = KinematicsSolverManager().catalog()[0].installed
        self.loading = True
        for combo, value in [
            (self.base, self.config.base_link),
            (self.end, self.config.end_effector),
        ]:
            combo.clear()
            combo.addItems(self.chain.model.links)
            combo.setCurrentText(value)
        self.gripper.clear()
        self.gripper.addItem("None / not assigned", "")
        for joint in self.chain.model.joints:
            if joint.movable and not joint.mimic:
                self.gripper.addItem(joint.name, joint.name)
        self.gripper.setCurrentIndex(
            max(
                0,
                self.gripper.findData(
                    self.config.gripper_joints[0] if self.config.gripper_joints else ""
                ),
            )
        )
        self.solver.setCurrentIndex(max(0, self.solver.findData(self.config.solver)))
        self.position_tolerance.setValue(self.config.options.position_tolerance * 1000)
        self.angle_tolerance.setValue(math.degrees(self.config.options.orientation_tolerance))
        self.starts.setValue(self.config.options.starts)
        self.populate_active(self.config.active_joints)
        if self.pose_editor:
            self.pose_holder.removeWidget(self.pose_editor)
            self.pose_editor.deleteLater()
        self.pose_editor = PoseEditor(self.chain, self.current())
        self.pose_editor.mode.setCurrentIndex(self.pose_editor.mode.findData(self.config.mode))
        self.pose_holder.addWidget(self.pose_editor)
        for spin in self.pose_editor.values:
            spin.valueChanged.connect(self.invalidate_ik)
        for combo in (
            self.pose_editor.mode,
            self.pose_editor.axis,
            self.pose_editor.frame,
            self.pose_editor.end,
        ):
            combo.currentIndexChanged.connect(self.invalidate_ik)
        while self.gripper_form.rowCount():
            self.gripper_form.removeRow(0)
        self.gripper_spins = {}
        for name in self.chain.gripper_joints:
            joint = self.engine.joints[name]
            factor = 1 if joint.kind == "prismatic" else 180 / math.pi
            spin = number(
                self.current()[name] * factor,
                (joint.lower if joint.lower is not None else -math.pi) * factor,
                (joint.upper if joint.upper is not None else math.pi) * factor,
                suffix=" m" if factor == 1 else " °",
            )
            self.gripper_spins[name] = (spin, factor)
            self.gripper_form.addRow(name, spin)
        self.chain_info.setText(
            f"{len(analysis.active_joints)} active arm joints\nMaximum sampled ranks: position {analysis.max_position_rank}/3 · pose {analysis.max_pose_rank}/6 · axis {analysis.max_axis_rank}/5\n"
            + "\n".join(analysis.notes)
        )
        self.kinematics_ready = bool(test and test.success and self.config.confirmed)
        self.solver_selected()
        status = (
            "Robot Kinematics: Ready · offline only"
            if self.kinematics_ready
            else "Review chain / gripper assignment and confirm"
            if self.was_installed
            else "Optional SciPy support is not installed"
        )
        self.ready.setText(
            status
            + f"\n{self.config.base_link} → {self.config.end_effector} · {len(self.chain.active_joints)} arm axes"
        )
        self.ik_output.setPlainText(
            test.message
            if test
            else "FK available. Install optional support and confirm a movable chain to enable IK."
        )
        self.backend.connect(self.chain.model, self.current(), self.chain.gripper_joints)
        self.loading = False
        self.refresh_positions()
        self.refresh_tasks()
        self.busy_changed(self.runner.busy)

    def current(self):
        return self.main.independent_positions()

    def populate_active(self, names):
        self.active.blockSignals(True)
        self.active.clear()
        eligible = self.engine.create_chain(
            self.base.currentText(),
            self.end.currentText(),
            gripper_joints=[self.gripper.currentData()] if self.gripper.currentData() else [],
        ).active_joints
        for name in eligible:
            item = QListWidgetItem(name, self.active)
            item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
            item.setCheckState(Qt.CheckState.Checked if name in names else Qt.CheckState.Unchecked)
        self.active.blockSignals(False)

    def chain_selection_changed(self, *_):
        if self.loading or not self.engine:
            return
        try:
            grippers = [self.gripper.currentData()] if self.gripper.currentData() else []
            provisional = self.engine.create_chain(
                self.base.currentText(), self.end.currentText(), gripper_joints=grippers
            )
            self.populate_active(provisional.active_joints)
        except ValueError as exc:
            self.ready.setText(str(exc))
        self.config_edited()

    def solver_selected(self, *_):
        identifier = self.solver.currentData()
        identifier = "scipy_trf" if identifier == "auto" else identifier
        descriptor = next(
            (d for d in KinematicsSolverManager().catalog() if d.id == identifier), None
        )
        status = (
            "Available · confirm chain to activate"
            if descriptor and descriptor.implemented and descriptor.installed
            else "Dependency missing · install Universal Kinematics Support"
            if descriptor and descriptor.implemented
            else "Not implemented · information only"
        )
        active = getattr(self, "config", None)
        self.solver_info.setText(
            f"Selected: {self.solver.currentText()} · {status}\nActive solver: {active.solver if active else 'none'}"
        )

    def config_edited(self, *_):
        if self.loading:
            return
        self.kinematics_ready = False
        self.ready.setText("Chain changed · confirm and run checks")
        self.invalidate_ik()
        self.invalidate_plan()
        self.main.mark_dirty()
        self.busy_changed(self.runner.busy)

    def options_edited(self, *_):
        if self.loading:
            return
        self.config.options.position_tolerance = self.position_tolerance.value() / 1000
        self.config.options.orientation_tolerance = math.radians(self.angle_tolerance.value())
        self.config.options.starts = self.starts.value()
        self.invalidate_ik()
        self.invalidate_plan()
        self.main.mark_dirty()

    def confirm_chain(self):
        if self.runner.busy or not self.engine:
            return
        self.solver_selected()
        candidate = self.solver.currentData()
        identifier = "scipy_trf" if candidate == "auto" else candidate
        descriptor = next(
            (d for d in KinematicsSolverManager().catalog() if d.id == identifier), None
        )
        if not descriptor or not descriptor.implemented or not descriptor.installed:
            return
        try:
            grippers = [self.gripper.currentData()] if self.gripper.currentData() else []
            active = [
                self.active.item(i).text()
                for i in range(self.active.count())
                if self.active.item(i).checkState() == Qt.CheckState.Checked
            ]
            chain = self.engine.create_chain(
                self.base.currentText(), self.end.currentText(), active, grippers
            )
            if not chain.active_joints:
                raise ValueError("Select at least one active arm joint")
        except ValueError as exc:
            self.error(str(exc))
            return
        self.cancel_work()
        self.generation += 1
        generation = self.generation
        config = self.config.model_copy(deep=True)
        config.base_link = chain.base_link
        config.end_effector = chain.end_effector
        config.active_joints = active
        config.gripper_joints = grippers
        config.confirmed = True
        config.solver = self.solver.currentData()
        current = self.current()
        self.cancel_event = threading.Event()

        def work(_):
            analysis = chain.analyze()
            test = KinematicsSolverManager().solve(
                chain,
                chain.forward_kinematics(current),
                current,
                solver=config.solver,
                options=config.options,
                cancel=self.cancel_event,
            )
            return self.engine, chain, config, analysis, test

        self.runner.run(work, lambda result: self.bound(result, generation))
        self.main.mark_dirty()

    def external_frame_changed(self, *_):
        if self.robot.loading or self.loading or not self.engine:
            return
        frame = self.robot.end_effector.currentText()
        if frame in self.engine.model.links and frame != self.config.end_effector:
            self.end.setCurrentText(frame)

    def robot_edited(self):
        if self.loading:
            return
        self.invalidate_ik()
        self.invalidate_plan()

    def invalidate_ik(self, *_):
        self.result = None
        self.previewing_ik = False
        if hasattr(self, "apply"):
            self.apply.setEnabled(False)
            self.preview.setEnabled(False)

    def invalidate_plan(self):
        self.cancel_work()
        self.plan = None
        if self.backend.status in ("playing", "paused"):
            self.stop()
        self.task_status.setText("Changed · revalidate before preview")

    def calculate_ik(self):
        if self.runner.busy or not self.chain or not self.kinematics_ready:
            return
        target = self.pose_editor.pose()
        current = self.current()
        chain = self.chain
        options = self.config.options.model_copy()
        solver = self.config.solver
        self.invalidate_ik()
        self.cancel_event = threading.Event()
        generation = self.generation
        token = target.model_dump_json()
        self.ik_output.setPlainText("Calculating IK in background…")

        def done(result):
            if (
                generation != self.generation
                or token != self.pose_editor.pose().model_dump_json()
                or current != self.current()
            ):
                self.ik_output.setPlainText("Context changed during calculation; result discarded.")
                return
            self.result = result
            self.ik_output.setPlainText(result.message)
            if result.best:
                self.ik_output.appendPlainText(
                    f"Task rank {result.best.rank} · condition {result.best.condition:.5g}\nCurrent-pose distance {result.best.distance:.5f} · {len(result.candidates)} evaluated candidates"
                )
            self.preview.setEnabled(result.success and self.main.mode == "offline")
            self.apply.setEnabled(result.success and self.main.mode == "offline")

        self.runner.run(
            lambda _: KinematicsSolverManager().solve(
                chain,
                target,
                current,
                solver=solver,
                mode=target.mode,
                options=options,
                axis=target.axis,
                cancel=self.cancel_event,
            ),
            done,
        )

    def preview_ik(self):
        if self.main.mode != "offline":
            self.error(
                "ROS viewer follows feedback only. Save the target and execute a simulation task."
            )
            return
        if self.result and self.result.success:
            self.robot.follow_live.blockSignals(True)
            self.robot.follow_live.setChecked(False)
            self.robot.follow_live.blockSignals(False)
            self.robot.view.set_joints(self.result.joint_positions)
            self.previewing_ik = True
            self.ik_output.appendPlainText(
                "TARGET PREVIEW ONLY · active state and monitoring unchanged. Apply explicitly to change the active robot state."
            )

    def apply_configuration(self, values):
        self.main.publish_offline(values)
        self.previewing_ik = False

    def apply_ik(self):
        if self.main.mode != "offline":
            self.error(
                "Apply is offline-only. Save the target and execute a validated ROS task instead."
            )
            return
        if self.result and self.result.success:
            values = self.result.joint_positions
            self.apply_configuration(values)
            self.main.mark_dirty()
            self.invalidate_plan()
            self.invalidate_ik()

    def reset_target(self):
        if not self.chain:
            return
        self.robot.view.set_joints(self.current())
        self.previewing_ik = False
        pose = self.positions.capture_cartesian("Current", self.chain, self.current()).pose
        self.pose_editor.set_pose(pose)
        self.invalidate_ik()

    def apply_gripper(self):
        if not self.chain:
            return
        try:
            q = dict(
                self.current(),
                **{
                    name: spin.value() / factor
                    for name, (spin, factor) in self.gripper_spins.items()
                },
            )
            self.apply_configuration(q)
            self.main.mark_dirty()
            self.invalidate_ik()
            self.invalidate_plan()
        except ValueError as exc:
            self.error(str(exc))

    def selected_position(self):
        row = self.position_table.currentRow()
        return self.positions.positions[row] if 0 <= row < len(self.positions.positions) else None

    def save_cartesian_target(self):
        if not self.chain or not self.pose_editor:
            return
        value = SavedPosition(
            name="Cartesian target", kind="cartesian", pose=self.pose_editor.pose()
        )
        dialog = PositionDialog(self.chain, self.current(), value, parent=self.main)
        if dialog.exec():
            try:
                self.positions.add(dialog.position())
                self.library_changed()
            except ValueError as exc:
                self.error(str(exc))

    def add_position(self, kind="cartesian"):
        if not self.chain:
            return
        dialog = PositionDialog(self.chain, self.current(), kind=kind, parent=self.main)
        if dialog.exec():
            try:
                self.positions.add(dialog.position())
                self.library_changed()
            except ValueError as exc:
                self.error(str(exc))

    def edit_position(self):
        value = self.selected_position()
        if not value or not self.chain:
            return
        dialog = PositionDialog(self.chain, self.current(), value, parent=self.main)
        if dialog.exec():
            try:
                self.positions.add(dialog.position())
                self.library_changed()
            except ValueError as exc:
                self.error(str(exc))

    def duplicate_position(self):
        value = self.selected_position()
        if value:
            self.positions.duplicate(value.id)
            self.library_changed()

    def delete_position(self):
        value = self.selected_position()
        if value:
            try:
                self.positions.remove(value.id, self.tasks)
                self.library_changed()
            except ValueError as exc:
                self.error(str(exc))

    def library_changed(self):
        self.refresh_positions()
        self.refresh_tasks()
        self.invalidate_plan()
        self.main.mark_dirty()

    def refresh_positions(self):
        self.position_table.setRowCount(len(self.positions.positions))
        for row, p in enumerate(self.positions.positions):
            state = (
                (
                    "Current robot"
                    if self.engine and p.model_signature == self.engine.signature
                    else "Different robot · re-teach"
                )
                if p.kind == "joint"
                else f"{p.pose.frame} → {p.pose.end_effector} · re-check IK"
            )
            for column, text in enumerate([p.name, p.kind, state]):
                self.position_table.setItem(row, column, QTableWidgetItem(text))

    def recall_position(self, apply):
        value = self.selected_position()
        if not value or not self.chain or self.runner.busy:
            return
        if self.main.mode != "offline":
            self.error("Recall is offline-only. Use a validated ROS task for this target.")
            return
        self.cancel_event = threading.Event()
        chain = self.chain
        positions = PositionManager(self.positions.positions)
        current = self.current()
        generation = self.generation
        cancel_event = self.cancel_event
        options, solver = self.config.options.model_copy(deep=True), self.config.solver

        def done(q):
            if generation != self.generation or cancel_event.is_set():
                return
            self.position_status.setText("Position validated for this robot · preview only")
            if apply:
                self.apply_configuration(q)
                self.main.mark_dirty()
                self.invalidate_ik()
                self.invalidate_plan()
            else:
                self.robot.view.set_joints(q)
                self.previewing_ik = True

        self.runner.run(
            lambda _: TaskCompiler(chain, positions, options, solver).resolve_position(
                value, current, cancel_event
            ),
            done,
        )

    def selected_task(self):
        identifier = self.task_choice.currentData()
        return next((task for task in self.tasks if task.id == identifier), None)

    def refresh_tasks(self, selected=None):
        current = selected or self.task_choice.currentData()
        self.task_choice.blockSignals(True)
        self.task_choice.clear()
        for task in self.tasks:
            self.task_choice.addItem(task.name, task.id)
        self.task_choice.setCurrentIndex(max(0, self.task_choice.findData(current)))
        self.task_choice.blockSignals(False)
        self.task_selected()

    def task_selected(self, *_):
        task = self.selected_task()
        self.steps.setRowCount(len(task.steps) if task else 0)
        self.speed.blockSignals(True)
        self.speed.setValue(round(task.settings.speed_scale * 100) if task else 25)
        self.speed.blockSignals(False)
        if task:
            for row, step in enumerate(task.steps):
                try:
                    target = self.positions.get(step.position_id).name if step.position_id else ""
                except ValueError:
                    target = "Missing position"
                for column, text in enumerate(
                    [step.name or f"Step {row + 1}", step.action, target, "Not validated"]
                ):
                    self.steps.setItem(row, column, QTableWidgetItem(text))
        if not self.loading:
            self.invalidate_plan()

    def new_task(self):
        name, ok = QInputDialog.getText(self.main, "New task", "Task name")
        if ok and name.strip():
            task = MotionTask(name=name.strip())
            self.tasks.append(task)
            self.refresh_tasks(task.id)
            self.main.mark_dirty()

    def rename_task(self):
        task = self.selected_task()
        if not task:
            return
        name, ok = QInputDialog.getText(self.main, "Rename task", "Task name", text=task.name)
        if ok and name.strip():
            task.name = name.strip()
            self.refresh_tasks(task.id)
            self.main.mark_dirty()

    def delete_task(self):
        task = self.selected_task()
        if (
            task
            and QMessageBox.question(self.main, "Delete task", f"Delete {task.name}?")
            == QMessageBox.StandardButton.Yes
        ):
            self.tasks = [t for t in self.tasks if t.id != task.id]
            self.refresh_tasks()
            self.main.mark_dirty()

    def add_step(self):
        task = self.selected_task()
        if not task or not self.chain:
            return
        dialog = StepDialog(self.chain, self.positions, self.current(), parent=self.main)
        if dialog.exec():
            task.steps.append(dialog.step())
            self.task_selected()
            self.main.mark_dirty()

    def edit_step(self):
        task = self.selected_task()
        row = self.steps.currentRow()
        if not task or not self.chain or not 0 <= row < len(task.steps):
            return
        dialog = StepDialog(self.chain, self.positions, self.current(), task.steps[row], self.main)
        if dialog.exec():
            task.steps[row] = dialog.step()
            self.task_selected()
            self.main.mark_dirty()

    def remove_step(self):
        task = self.selected_task()
        row = self.steps.currentRow()
        if task and 0 <= row < len(task.steps):
            task.steps.pop(row)
            self.task_selected()
            self.main.mark_dirty()

    def move_step(self, delta):
        task = self.selected_task()
        row = self.steps.currentRow()
        target = row + delta
        if task and 0 <= row < len(task.steps) and 0 <= target < len(task.steps):
            task.steps[row], task.steps[target] = task.steps[target], task.steps[row]
            self.task_selected()
            self.steps.selectRow(target)
            self.main.mark_dirty()

    def speed_changed(self, *_):
        task = self.selected_task()
        if not self.loading and task:
            task.settings.speed_scale = self.speed.value() / 100
            self.invalidate_plan()
            self.main.mark_dirty()

    def compile_task(self, play=False, single=None):
        task = self.selected_task()
        if not task or not self.chain or self.runner.busy:
            return
        if not self.kinematics_ready:
            self.error("Confirm a ready offline chain before compiling a task")
            return
        source = task.model_copy(deep=True)
        if single is not None:
            source.steps = [source.steps[single]]
            source.name += " · selected step"
        self.stop()
        self.cancel_event = threading.Event()
        cancel_event = self.cancel_event
        generation = self.generation
        current = self.current()
        chain = self.chain
        compiler = TaskCompiler(
            chain,
            PositionManager(self.positions.positions),
            self.config.options.model_copy(),
            self.config.solver,
        )

        def done(plan):
            if generation != self.generation or cancel_event.is_set():
                return
            self.plan = plan
            if not plan.valid:
                for index, message in plan.errors.items():
                    row = single if single is not None else index
                    if row < self.steps.rowCount():
                        item = QTableWidgetItem("ERROR: " + message)
                        item.setForeground(QColor("#ff8b8b"))
                        item.setToolTip(message)
                        self.steps.setItem(row, 3, item)
                self.task_status.setText(
                    "Validation failed · " + next(iter(plan.errors.values()), "Empty task")
                )
                return
            for index, step in enumerate(plan.steps):
                row = single if single is not None else index
                if row < self.steps.rowCount():
                    self.steps.setItem(
                        row, 3, QTableWidgetItem(f"{step.trajectory.duration:.2f} s · valid")
                    )
            self.task_status.setText(
                f"Kinematically validated · {plan.duration:.2f} s\n" + ("\n".join(plan.notes[:3]))
            )
            if plan.object_initial is not None:
                self.robot.view.set_virtual_object(
                    plan.object_initial, plan.source.virtual_object.size
                )
            if play and self.main.mode == "ros":
                try:
                    self.main.ros_execution.execute(plan)
                except (ValueError, ConnectionError) as exc:
                    self.error(str(exc))
                return
            if play:
                self.backend.connect(chain.model, current, chain.gripper_joints)
                self.backend.execute(plan)
                self.last_tick = time.monotonic()
                self.robot.joint_controls.setEnabled(False)
                self.robot.follow_live.blockSignals(True)
                self.robot.follow_live.setChecked(False)
                self.robot.follow_live.blockSignals(False)
                self.main.mark_dirty()

        self.runner.run(
            lambda progress: compiler.compile(source, current, cancel_event, progress), done
        )

    def choose_backend(self, index):
        if index == 1 and self.main.mode != "ros":
            self.main.start_ros()
        elif index == 0 and self.main.mode == "ros":
            self.main.request_offline()
        # Selection reflects the actual completed mode switch.
        self.backend_choice.blockSignals(True)
        self.backend_choice.setCurrentIndex(1 if self.main.mode == "ros" else 0)
        self.backend_choice.blockSignals(False)

    def compile_selected(self):
        row = self.steps.currentRow()
        if row >= 0:
            self.compile_task(False, row)

    def pause(self):
        if self.main.mode != "offline":
            self.error("ROS action pause is unavailable; Stop cancels the simulation")
            return
        if self.backend.status == "playing":
            self.backend.pause()
            state = self.backend.read_state()
            self.main.publish_offline(
                state.joints,
                velocities=state.velocities,
                accelerations=state.accelerations,
                source="offline trajectory",
                status="paused",
            )
        elif self.backend.status == "paused":
            self.last_tick = time.monotonic()
            self.backend.resume()

    def stop(self):
        if hasattr(self.main, "ros_execution") and self.main.mode == "ros":
            self.main.ros_execution.abort()
        self.cancel_work()
        self.backend.abort()
        self.robot.joint_controls.setEnabled(self.main.mode == "offline")
        state = self.main.state_manager.snapshot()
        if state and self.main.mode == "offline" and self.backend.model is self.main.model:
            self.main.publish_offline(
                self.current(),
                velocities={n: 0.0 for n in self.current()} if self.backend.plan else None,
                source="offline trajectory" if self.backend.plan else "manual",
                status="stopped",
            )

    def reset(self):
        if self.main.mode != "offline":
            self.error("ROS feedback cannot be reset locally; cancel and replan from current state")
            return
        self.backend.reset()
        if self.backend.model is self.main.model and self.backend.plan:
            self.apply_configuration(self.backend.read_joint_positions())
            state = self.backend.read_state()
            self.robot.view.set_virtual_object(
                state.object_transform,
                self.backend.plan.source.virtual_object.size
                if self.backend.plan.source.virtual_object
                else 0.02,
            )
        self.robot.joint_controls.setEnabled(True)
        self.progress.setValue(0)

    def tick(self):
        if self.main.closed:
            return
        now = time.monotonic()
        delta = now - self.last_tick
        self.last_tick = now
        if self.chain:
            try:
                pose = pose_rpy(self.chain.forward_kinematics(self.current()))
                self.current_pose.setText(
                    f"{self.chain.base_link} → {self.chain.end_effector}\nX {pose[0] * 1000:+.2f} · Y {pose[1] * 1000:+.2f} · Z {pose[2] * 1000:+.2f} mm\nR {math.degrees(pose[3]):+.2f}° · P {math.degrees(pose[4]):+.2f}° · Y {math.degrees(pose[5]):+.2f}°"
                )
            except (ValueError, KeyError):
                pass
        if self.backend.status == "playing" and self.main.mode == "offline":
            state = self.backend.advance(delta)
            self.main.publish_offline(
                state.joints,
                velocities=state.velocities,
                accelerations=state.accelerations,
                source="offline trajectory",
                status=state.status,
                progress=state.elapsed / self.backend.plan.duration
                if self.backend.plan.duration
                else 1.0,
            )
            self.robot.view.set_virtual_object(
                state.object_transform,
                self.backend.plan.source.virtual_object.size
                if self.backend.plan.source.virtual_object
                else 0.02,
            )
            self.progress.setValue(
                round(state.elapsed / self.backend.plan.duration * 100)
                if self.backend.plan.duration
                else 100
            )
            self.task_status.setText(
                f"{state.status} · step {state.step + 1}/{len(self.backend.plan.steps)} · {state.elapsed:.2f}/{self.backend.plan.duration:.2f} s\nCube: {'attached visually' if state.attached else 'detached'} · "
                + COLLISION_NOTICE
            )
            self.steps.selectRow(state.step)
            if state.status == "finished":
                self.robot.joint_controls.setEnabled(True)

    def open_wizard(self):
        if not self.chain or not self.kinematics_ready:
            return
        if self.main.mode != "offline":
            self.error(
                "Teach / generate tasks in Offline Preview, then execute through ROS Simulation."
            )
            return
        if not self.chain.gripper_joints:
            self.error("Assign a gripper joint in Chain setup and confirm first")
            return
        signature = self.engine.signature

        def generated(result):
            if not self.engine or self.engine.signature != signature:
                self.error("Robot changed; generated task discarded")
                return
            positions, task, plan = result
            for position in positions:
                self.positions.add(position)
            self.tasks.append(task)
            self.refresh_positions()
            self.refresh_tasks(task.id)
            self.plan = plan
            self.program_tabs.setCurrentIndex(1)
            self.main.mark_dirty()
            self.task_status.setText(
                f"Wizard validated {len(task.steps)} steps · {plan.duration:.2f} s · preview only"
            )
            self.robot.detail_tabs.setCurrentWidget(self.program_panel)
            self.robot.view.set_virtual_object(plan.object_initial, task.virtual_object.size)

        self.wizard = PickPlaceWizard(
            self.chain,
            self.current(),
            PositionManager(self.positions.positions),
            self.main.project.motion,
            self.config.options,
            self.config.solver,
            generated,
            self.main,
        )
        initial = self.current()

        def preview(plan):
            if self.engine.signature != signature:
                return
            self.stop()
            self.backend.connect(self.chain.model, initial, self.chain.gripper_joints)
            self.backend.execute(plan)
            self.last_tick = time.monotonic()
            self.robot.joint_controls.setEnabled(False)
            self.robot.follow_live.blockSignals(True)
            self.robot.follow_live.setChecked(False)
            self.robot.follow_live.blockSignals(False)

        self.wizard.preview_callback = preview

        def finished(result):
            self.stop()
            self.apply_configuration(initial)
            self.robot.view.set_virtual_object(None)

        self.wizard.finished.connect(finished)
        self.wizard.setWindowModality(Qt.WindowModality.WindowModal)
        self.wizard.show()
        self.wizard.move(self.main.mapToGlobal(self.robot.detail_tabs.pos()))

    def collect(self, project):
        if self.pose_editor:
            self.config.mode = self.pose_editor.mode.currentData()
        project.kinematics = self.config.model_copy(deep=True)
        project.positions = [p.model_copy(deep=True) for p in self.positions.positions]
        project.tasks = [t.model_copy(deep=True) for t in self.tasks]

    def export_library(self):
        path, _ = QFileDialog.getSaveFileName(
            self.main, "Export motion library", "motion-library.yaml", "YAML (*.yaml)"
        )
        if not path:
            return
        data = dict(
            schema_version=1,
            positions=[p.model_dump(mode="json") for p in self.positions.positions],
            tasks=[t.model_dump(mode="json") for t in self.tasks],
        )
        self.runner.run(
            lambda _: atomic_text(
                Path(path), yaml.safe_dump(data, sort_keys=False, allow_unicode=True)
            ),
            lambda _: self.task_status.setText("Motion library exported"),
        )

    def import_library(self):
        path, _ = QFileDialog.getOpenFileName(
            self.main, "Import motion library", "", "YAML (*.yaml *.yml)"
        )
        if not path:
            return

        def work(_):
            source = Path(path)
            if source.stat().st_size > 2_000_000:
                raise ValueError("Motion library exceeds 2 MB")
            data = yaml.safe_load(source.read_text())
            if (
                not isinstance(data, dict)
                or set(data) - {"schema_version", "positions", "tasks"}
                or data.get("schema_version") != 1
            ):
                raise ValueError("Unsupported motion library schema")
            return [SavedPosition.model_validate(p) for p in data.get("positions", [])], [
                MotionTask.model_validate(t) for t in data.get("tasks", [])
            ]

        def done(result):
            positions, tasks = result
            candidate = PositionManager(self.positions.positions)
            try:
                for p in positions:
                    candidate.add(p)
            except ValueError as exc:
                self.error(str(exc))
                return
            existing = {t.id: t for t in self.tasks}
            existing.update({t.id: t for t in tasks})
            self.positions = candidate
            self.tasks = list(existing.values())
            self.library_changed()

        self.runner.run(work, done)

    def error(self, message):
        self.ik_output.appendPlainText("ERROR: " + message)
        self.task_status.setText("ERROR: " + message)
        self.position_status.setText(message)

    def busy_changed(self, busy):
        ready = getattr(self, "kinematics_ready", False)
        self.calculate.setEnabled(not busy and ready)
        self.cancel.setEnabled(busy)
        self.confirm.setEnabled(not busy and self.engine is not None)
        self.install_button.setVisible(not KinematicsSolverManager().catalog()[0].installed)
        for widget in (
            self.validate_task_button,
            self.single_step_button,
            self.play_button,
            self.wizard_button,
        ):
            widget.setEnabled(not busy and ready)
        self.preview.setEnabled(
            not busy and self.main.mode == "offline" and bool(self.result and self.result.success)
        )
        self.apply.setEnabled(
            not busy and self.main.mode == "offline" and bool(self.result and self.result.success)
        )
        if hasattr(self.main, "backend_workspace"):
            self.main.backend_workspace.refresh_selector()

    def cancel_work(self):
        self.cancel_event.set()

    def install_support(self):
        self.main.open_wizard()
        if self.main.wizard:
            self.main.wizard.setStartId(3)
            self.main.wizard.restart()

    def shutdown(self):
        self.cancel_work()
        self.backend.disconnect()
        self.timer.stop()
        if self.wizard:
            self.wizard.reject()
