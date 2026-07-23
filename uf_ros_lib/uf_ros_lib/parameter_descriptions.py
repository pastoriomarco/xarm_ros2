
#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2024, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
 
import yaml
from launch_ros.parameter_descriptions import ParameterValue
from launch.utilities.type_utils import perform_typed_substitution

class YamlParameterValue(ParameterValue):
    def __str__(self):
        return 'uf_ros_lib.parameter_descriptions.YamlParameterValue(value={}, value_type={})'.format(self.value, self.value_type)

    def update(self, other):
        """Defer a shallow YAML mapping merge until launch evaluation.

        MoveItConfigs.to_dict() merges Pilz cartesian limits into
        robot_description_planning with ``dict.update``. Deferred builder
        values are ParameterValue instances rather than dictionaries, so keep
        the overlay here and apply it after both YAML substitutions resolve.
        """
        if not hasattr(self, '_yaml_overlays'):
            self._yaml_overlays = []
        if all(overlay is not other for overlay in self._yaml_overlays):
            self._yaml_overlays.append(other)

    def evaluate(self, context):
        """Evaluate and return parameter rule."""
        self.__evaluated_parameter_value = perform_typed_substitution(
            context, self.value, self.value_type)
        self.__evaluated_parameter_value = yaml.safe_load(self.__evaluated_parameter_value)
        for overlay in getattr(self, '_yaml_overlays', []):
            if hasattr(overlay, 'evaluate'):
                overlay = overlay.evaluate(context)
            if overlay:
                self.__evaluated_parameter_value.update(overlay)
        return self.__evaluated_parameter_value
