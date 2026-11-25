# Copyright 2026 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Loading of Nebula parameter files outside of a ROS 2 launch context."""

from pathlib import Path
import re
from typing import Any
from typing import Dict
from typing import Optional
from typing import Union

from ament_index_python.packages import get_package_share_directory
import yaml

_SUBSTITUTION = re.compile(r"\$\((find-pkg-share|var)\s+([^)\s]+)\)")


def load_parameters(
    path: Union[str, Path], overrides: Optional[Dict[str, Any]] = None
) -> Dict[str, Any]:
    """Load the parameters of a sensor from a Nebula parameter file.

    The ``/**: ros__parameters:`` root is removed, and the launch substitutions that Nebula's
    parameter files use are resolved in string values: ``$(find-pkg-share <package>)``, and
    ``$(var <name>)`` with the value of the top-level parameter ``<name>``.

    Args:
        path: The parameter file, e.g. ``nebula_hesai/config/Pandar128E4X.param.yaml``.
        overrides: Top-level parameters to set or replace before substitutions are resolved.

    Returns:
        The parameters as a nested dict, as the decoders in this package accept them.

    Raises:
        KeyError: If a ``$(var <name>)`` substitution refers to a parameter that does not exist.
    """
    with open(path, encoding="utf-8") as f:
        parameters = yaml.safe_load(f)

    for key in ("/**", "ros__parameters"):
        if isinstance(parameters, dict) and set(parameters) == {key}:
            parameters = parameters[key]

    parameters.update(overrides or {})
    return _substitute(parameters, parameters)


def _substitute(value: Any, parameters: Dict[str, Any]) -> Any:
    if isinstance(value, dict):
        return {k: _substitute(v, parameters) for k, v in value.items()}
    if isinstance(value, list):
        return [_substitute(v, parameters) for v in value]
    if not isinstance(value, str):
        return value

    def replace(match: "re.Match[str]") -> str:
        kind, argument = match.groups()
        if kind == "find-pkg-share":
            return get_package_share_directory(argument)
        return str(parameters[argument])

    return _SUBSTITUTION.sub(replace, value)
