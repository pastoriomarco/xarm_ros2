#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2026, Marco Pastorio
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Regression tests for the bounded XC-4 upstream integration."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
import pytest

from uf_ros_lib.moveit_configs_builder import DualMoveItConfigsBuilder
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.moveit_configs_builder import TripleMoveItConfigsBuilder
from uf_ros_lib.parameter_descriptions import YamlParameterValue


SINGLE_ARGUMENTS = {
    'prefix': 'A_',
    'robot_type': 'lite',
    'dof': '6',
    'add_gripper': 'true',
    'add_bio_gripper': 'false',
}
XARM_ARGUMENTS = {
    'prefix': 'X_',
    'robot_type': 'xarm',
    'dof': '7',
    'add_gripper': 'false',
    'add_bio_gripper': 'false',
}
DUAL_ARGUMENTS = {
    'prefix_1': 'A_',
    'prefix_2': 'B_',
    'robot_type_1': 'lite',
    'robot_type_2': 'xarm',
    'dof_1': '6',
    'dof_2': '7',
    'add_gripper_1': 'true',
    'add_gripper_2': 'true',
    'add_bio_gripper_1': 'false',
    'add_bio_gripper_2': 'false',
}
TRIPLE_ARGUMENTS = {
    'prefix_1': 'A_',
    'prefix_2': 'B_',
    'prefix_3': 'C_',
    'robot_type_1': 'lite',
    'robot_type_2': 'xarm',
    'robot_type_3': 'lite',
    'dof_1': '6',
    'dof_2': '7',
    'dof_3': '6',
    'add_gripper_1': 'true',
    'add_gripper_2': 'true',
    'add_gripper_3': 'true',
    'add_bio_gripper_1': 'false',
    'add_bio_gripper_2': 'false',
    'add_bio_gripper_3': 'false',
}


def _evaluate(value, context):
    """Resolve a deferred launch value, leaving static values intact."""
    if isinstance(value, dict):
        return {
            key: _evaluate(item, context)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [_evaluate(item, context) for item in value]
    if isinstance(value, YamlParameterValue):
        return value.evaluate(context)
    performer = getattr(value, 'perform', None)
    if callable(performer):
        return performer(context)
    return value


def _normalize_xml(value):
    """Normalize generated XML for static/deferred comparisons."""
    root = ET.fromstring(value)
    for element in root.iter():
        if element.text is not None and not element.text.strip():
            element.text = None
        if element.tail is not None and not element.tail.strip():
            element.tail = None
        element.attrib.update(sorted(element.attrib.items()))
    return ET.tostring(root, encoding='unicode')


def _selected_config(builder_class, arguments, deferred):
    """Build the XC-4 YAML surfaces through one builder dispatch path."""
    context = LaunchContext()
    context.launch_configurations.update(arguments)
    context.launch_configurations['controllers_name'] = 'fake_controllers'

    if deferred:
        builder_arguments = {
            name: LaunchConfiguration(name)
            for name in arguments
        }
        controllers_name = LaunchConfiguration('controllers_name')
    else:
        builder_arguments = arguments
        controllers_name = 'fake_controllers'

    builder = builder_class(
        context=None,
        controllers_name=controllers_name,
        **builder_arguments,
    )
    config = (
        builder
        .joint_limits()
        .trajectory_execution()
        .planning_pipelines()
        .to_moveit_configs()
    )
    values = _evaluate(config.to_dict(), context)
    return {
        'urdf': _normalize_xml(
            values['robot_description']
        ),
        'srdf': _normalize_xml(
            values['robot_description_semantic']
        ),
        'joint_limits': values['robot_description_planning'],
        'controllers': values['moveit_simple_controller_manager'],
        'planning_pipelines': values['planning_pipelines'],
        'ompl': values['ompl'],
    }


@pytest.mark.parametrize(
    ('builder_class', 'arguments'),
    [
        (MoveItConfigsBuilder, SINGLE_ARGUMENTS),
        (MoveItConfigsBuilder, XARM_ARGUMENTS),
        (DualMoveItConfigsBuilder, DUAL_ARGUMENTS),
        (TripleMoveItConfigsBuilder, TRIPLE_ARGUMENTS),
    ],
)
def test_static_and_truly_deferred_builder_outputs_match(
    builder_class,
    arguments,
):
    """Require no-context LaunchConfigurations to match static YAML output."""
    static = _selected_config(builder_class, arguments, deferred=False)
    deferred = _selected_config(builder_class, arguments, deferred=True)

    assert deferred == static


def test_lite_and_xarm_gripper_profiles_are_selected_per_instance():
    """Keep mixed dual gripper configuration isolated by robot type."""
    config = _selected_config(
        DualMoveItConfigsBuilder,
        DUAL_ARGUMENTS,
        deferred=True,
    )
    limits = config['joint_limits']['joint_limits']
    controllers = config['controllers']
    ompl = config['ompl']

    assert 'A_left_finger_joint' in limits
    assert 'A_right_finger_joint' in limits
    assert 'A_drive_joint' not in limits
    assert 'B_drive_joint' in limits
    assert 'A_lite_gripper_controller' in controllers
    assert controllers['A_lite_gripper_controller']['joints'] == [
        'A_right_finger_joint',
    ]
    assert 'B_xarm_gripper_traj_controller' in controllers
    assert controllers['B_xarm_gripper_traj_controller']['joints'] == [
        'B_drive_joint',
    ]
    assert 'A_lite_gripper' in ompl
    assert 'B_xarm_gripper' in ompl


def test_working_lite_gripper_profile_remains_selected():
    """Protect the maintained master/mimic Lite profile from silent replacement."""
    builder = MoveItConfigsBuilder(
        context=None,
        controllers_name='fake_controllers',
        **SINGLE_ARGUMENTS,
    )
    urdf = builder.to_moveit_configs().robot_description[
        'robot_description'
    ]
    root = ET.fromstring(urdf)
    right_joint = root.find("./joint[@name='A_right_finger_joint']")
    left_joint = root.find("./joint[@name='A_left_finger_joint']")

    assert right_joint is not None
    assert right_joint.find('mimic') is None
    assert left_joint is not None
    mimic = left_joint.find('mimic')
    assert mimic is not None
    assert mimic.attrib == {
        'joint': 'A_right_finger_joint',
        'multiplier': '1',
        'offset': '0',
    }

    description_share = Path(
        get_package_share_directory('xarm_description')
    )
    assert (
        description_share
        / 'urdf'
        / 'gripper'
        / 'lite_gripper_macro.xacro'
    ).is_file()


def test_selected_upstream_correctness_fixes_are_present():
    """Retain only the audited independent fixes from current upstream."""
    moveit_share = Path(get_package_share_directory('xarm_moveit_config'))
    description_share = Path(get_package_share_directory('xarm_description'))

    camera = (
        description_share / 'urdf' / 'camera' / 'camera.gazebo.xacro'
    ).read_text(encoding='utf-8')
    device = (
        description_share / 'urdf' / 'xarm_device_macro.xacro'
    ).read_text(encoding='utf-8')
    bio_ompl = (
        moveit_share / 'config' / 'bio_gripper' / 'ompl_planning.yaml'
    ).read_text(encoding='utf-8')
    triple_real = (
        moveit_share
        / 'launch'
        / '_triple_robot_moveit_realmove.launch.py'
    ).read_text(encoding='utf-8')

    assert '<format>B8G8R8</format>' in camera
    assert '<foramt>' not in camera
    assert 'bio_gripper:' in bio_ompl
    assert 'bios_gripper:' not in bio_ompl
    assert '<xacro:if value="${is_ros2}">' in device
    assert '<xacro:unless value="${is_ros2}">' in device
    assert "'{}{}_traj_controller'.format(prefix_3.perform(context)," in (
        triple_real
    )
