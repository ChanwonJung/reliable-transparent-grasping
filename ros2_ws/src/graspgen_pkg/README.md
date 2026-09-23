# graspgen_pkg

EE depth + GSAM 마스크 → GraspGen 원격 추론 → `/grasp_candidates` 발행.  
`/grasp_candidates` JSON 형식은 `bt_pkg`가 기대하는 스키마 그대로 — 코드 변경 불필요.

---

## 요구사항

### 구독 토픽 (아래 토픽이 발행되고 있어야 동작)

| 토픽 | 발행 노드 |
|---|---|
| `/ee_camera/depth_image` | Isaac Sim |
| `/ee_camera/camera_info` | Isaac Sim |
| `/sam/mask_image` | `sam_mask_node` (legacy: `qwen_bridge_node`) |
| `/qwen/labeled_detections` | `qwen_bridge_node` |
| `/world_map_result` | `multi_view_projector_node` **(트리거)** |

### Python 패키지

repo `.venv` 에 설치 (`requirements/ros2_runtime.txt`):

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements/ros2_runtime.txt
```

### SSH 터널 (노드 실행 전 개통)

`environments/ros2/seraph_env.sh` 가 aurora.khu.ac.kr:30080 jump host 경유로
aurora-g6 (Seraph) 터널을 연다 (ROS + venv 환경도 같이 설정):

| 서비스 | localhost 포트 |
|---|---|
| Qwen (SGLang) | 10000 |
| SAM 2.1 | 20000 |
| SwinDRNet | 30000 |
| GraspGen (ZMQ REP) | 40000 |

```bash
source environments/ros2/seraph_env.sh
```

---

## 빌드

```bash
source environments/ros2/seraph_env.sh --no-tunnel
cd ros2_ws && colcon build --packages-select graspgen_pkg
```

## 실행

```bash
ros2 launch graspgen_pkg graspgen.launch.py
```

카메라 포즈는 기본적으로 TF (`extrinsics_source:=tf`). Isaac 정적 R/t 를 쓰려면
`extrinsics_source:=yaml extrinsics_config:=<mask_projection_pkg 와 같은 YAML>`.

---

## 발행 토픽

| 토픽 | 소비자 |
|---|---|
| `/grasp_candidates` | `bt_executor_node` |
| `/grasp_markers` | RViz2 |
| `/graspgen/target_cloud` | 디버그 |

### `/grasp_candidates` JSON 형식

```json
{
  "candidates": [
    {
      "position":   [x, y, z],
      "quaternion": [qx, qy, qz, qw],
      "width":      0.08,
      "quality":    0.92,
      "frame":      "panda_link0"
    }
  ],
  "target_centroid": [x, y, z],
  "stamp": 1234567890.123
}
```

`candidates`는 `quality` 내림차순 정렬. TF(`world` → `panda_link0`) 실패 시 `frame`은 `"world"`로 발행.

---

## 주의사항

- `topk_num_grasps`(기본 5) = `robot_defaults.yaml`의 `max_grasp_candidates` 값과 반드시 일치시킬 것
- `target_pose_bridge_node`와 동시에 켜면 `/grasp_candidates` 중복 발행 → 둘 중 하나만 실행
- 터널 없이 실행하면 5초 후 timeout 오류 발생
