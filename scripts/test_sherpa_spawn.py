#!/usr/bin/env python3
"""셰르파 최소 URDF Gazebo 스폰 테스트.

사용법 (WSL):
  source /opt/ros/jazzy/setup.bash && cd ~/ros2_ws && source install/setup.bash
  python3 /mnt/c/Users/USER/Desktop/SUPERAGENT/projects/argos/scripts/test_sherpa_spawn.py
"""
import subprocess, time, sys, yaml, os

URDF_MINIMAL = """<?xml version="1.0"?>
<robot name="sherpa_min">
  <link name="base_footprint"/>
  <link name="base_link">
    <visual><geometry><box size="3.1 2.0 1.0"/></geometry></visual>
    <collision><geometry><box size="3.1 2.0 1.0"/></geometry></collision>
    <inertial><mass value="500.0"/>
    <inertia ixx="200" ixy="0" ixz="0" iyy="400" iyz="0" izz="500"/></inertial>
  </link>
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/><child link="base_link"/>
    <origin xyz="0 0 0.85"/>
  </joint>
</robot>"""

def main():
    # 1. YAML 파라미터 파일 생성
    params = {
        "robot_state_publisher": {
            "ros__parameters": {
                "robot_description": URDF_MINIMAL,
                "use_sim_time": True,
            }
        }
    }
    yaml_path = "/tmp/sherpa_test_rsp.yaml"
    with open(yaml_path, "w") as f:
        yaml.dump(params, f, default_flow_style=False, allow_unicode=True)
    print(f"[OK] YAML written: {yaml_path}")

    # 2. RSP 실행
    rsp = subprocess.Popen(
        ["ros2", "run", "robot_state_publisher", "robot_state_publisher",
         "--ros-args", "--params-file", yaml_path],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
    )
    time.sleep(3)
    if rsp.poll() is not None:
        print(f"[FAIL] RSP died: {rsp.stderr.read().decode()[:200]}")
        return 1
    print("[OK] RSP running")

    # 3. robot_description 토픽 확인
    result = subprocess.run(
        ["ros2", "topic", "info", "/robot_description"],
        capture_output=True, text=True, timeout=5,
    )
    print(f"[INFO] {result.stdout.strip()}")

    # 4. Gazebo 스폰
    spawn = subprocess.run(
        ["ros2", "run", "ros_gz_sim", "create",
         "-world", "indoor_test", "-name", "sherpa_test",
         "-topic", "robot_description", "-x", "2", "-y", "2", "-z", "2"],
        capture_output=True, text=True, timeout=30,
    )
    print(f"[SPAWN] {spawn.stdout.strip()}")
    if "successful" in spawn.stdout.lower():
        print("[OK] 셰르파 스폰 성공!")
    else:
        print(f"[FAIL] {spawn.stderr.strip()[:200]}")

    # 5. 모델 확인
    time.sleep(3)
    models = subprocess.run(["gz", "model", "--list"], capture_output=True, text=True, timeout=10)
    print(f"[MODELS] {models.stdout.strip()}")

    rsp.terminate()
    return 0

if __name__ == "__main__":
    sys.exit(main())
