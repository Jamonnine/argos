# Copyright 2026 민발 (Minbal), 대구강북소방서
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
"""pytest conftest — argos_bringup 패키지 경로 자동 등록.

ROS2 colcon 빌드 없이도 argos_bringup 모듈을 직접 임포트할 수 있도록
sys.path에 소스 디렉토리를 추가한다.

적용 범위: 이 conftest.py가 위치한 test/ 디렉토리의 모든 테스트 파일.
"""
import sys
import pathlib

# argos_bringup 소스 루트: ros2_ws/src/argos_bringup/
_PACKAGE_ROOT = pathlib.Path(__file__).parent.parent

# sys.path에 없는 경우에만 추가 (중복 방지)
_package_root_str = str(_PACKAGE_ROOT)
if _package_root_str not in sys.path:
    sys.path.insert(0, _package_root_str)
