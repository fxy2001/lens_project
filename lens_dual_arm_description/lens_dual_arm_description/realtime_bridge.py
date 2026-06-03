from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from motor_can_protocol import MitRanges, format_multi_frames, pack_std_mit_non_broadcast


@dataclass(frozen=True)
class JointTarget:
    joint_name: str
    position: float
    velocity: float
    torque: float
    kp: float
    kd: float


class RealTimeBridge:
    """
    仿真控制输出桥接层：
    - protocol='can': 输出逐关节 CAN 帧（MIT 0x0B）
    - protocol='ethercat': 输出与 actuator_SDK 接口语义一致的批量 MIT 命令
    """

    def __init__(
        self,
        *,
        protocol: str,
        left_joint_names: list[str],
        left_can_ids: list[int],
        print_hz: float,
        mit_ranges: MitRanges,
        mit_vel: float,
        mit_kp: float,
        mit_kd: float,
        mit_tor: float,
        ethercat_ifname: str = "enp3s0",
        actuator_config_path: str = "",
        actuator_params_path: str = "",
        ethercat_send_frequency: int = 500,
    ) -> None:
        self.protocol = str(protocol).strip().lower()
        self.left_joint_names = list(left_joint_names)
        self.left_can_ids = list(left_can_ids)
        self.print_hz = float(print_hz)
        self.mit_ranges = mit_ranges
        self.mit_vel = float(mit_vel)
        self.mit_kp = float(mit_kp)
        self.mit_kd = float(mit_kd)
        self.mit_tor = float(mit_tor)
        self.ethercat_ifname = str(ethercat_ifname)
        self.actuator_config_path = actuator_config_path
        self.actuator_params_path = actuator_params_path
        self.ethercat_send_frequency = int(ethercat_send_frequency)
        self._last_t = 0.0
        self._ec_init_printed = False
        self._ec_slave_port = self._load_slave_port_map(actuator_config_path)

        if len(self.left_joint_names) != 7:
            raise ValueError("left_joint_names must have 7 entries")
        if len(self.left_can_ids) != 7:
            raise ValueError("left_can_ids must have 7 entries")
        if self.protocol not in ("can", "ethercat"):
            raise ValueError("protocol must be 'can' or 'ethercat'")

    @staticmethod
    def _load_slave_port_map(config_path: str) -> dict[str, tuple[int, int]]:
        """
        从 actuator_config.json 读取 joint_name -> (ethercat_slave_id, port)。
        """
        if not config_path:
            return {}
        p = Path(config_path)
        if not p.exists():
            return {}
        try:
            cfg = json.loads(p.read_text(encoding="utf-8"))
            out: dict[str, tuple[int, int]] = {}
            for node in cfg.get("acutator_list", []):
                slave = int(node.get("ethercat_slave_id", -1))
                for item in node.get("can_id_list", []):
                    j = str(item.get("joint_name", ""))
                    port = int(item.get("port", -1))
                    if j:
                        out[j] = (slave, port)
            return out
        except Exception:
            return {}

    def maybe_emit(self, now_s: float, q_cmd: np.ndarray) -> None:
        if self.print_hz <= 0:
            return
        if now_s - self._last_t < 1.0 / self.print_hz:
            return
        self._last_t = now_s
        q = np.asarray(q_cmd, dtype=float).reshape(-1)
        targets = [
            JointTarget(
                joint_name=j,
                position=float(p),
                velocity=self.mit_vel,
                torque=self.mit_tor,
                kp=self.mit_kp,
                kd=self.mit_kd,
            )
            for j, p in zip(self.left_joint_names, q)
        ]
        if self.protocol == "can":
            self._emit_can(targets)
        else:
            self._emit_ethercat(targets)

    def _emit_can(self, targets: list[JointTarget]) -> None:
        frames = []
        for can_id, t in zip(self.left_can_ids, targets):
            payload = pack_std_mit_non_broadcast(
                pos=t.position,
                vel=t.velocity,
                kp=t.kp,
                kd=t.kd,
                tor=t.torque,
                ranges=self.mit_ranges,
            )
            frames.append((can_id, payload))
        print("\n[Bridge TX] protocol=CAN MIT CMD=0x0B", flush=True)
        print(format_multi_frames(frames), flush=True)

    def _emit_ethercat(self, targets: list[JointTarget]) -> None:
        # 按 actuator_SDK::setTargetMit(vector<joint_names>, ... ) 语义打印
        names = [t.joint_name for t in targets]
        pos = [t.position for t in targets]
        vel = [t.velocity for t in targets]
        tor = [t.torque for t in targets]
        kp = [t.kp for t in targets]
        kd = [t.kd for t in targets]
        if not self._ec_init_printed:
            self._ec_init_printed = True
            print("\n[Bridge INIT] protocol=EtherCAT actuator_SDK flow", flush=True)
            if self.actuator_params_path:
                print(
                    "ActuatorController::loadActuatorParamsConfig("
                    f"\"{self.actuator_params_path}\")",
                    flush=True,
                )
            else:
                print("ActuatorController::loadActuatorParamsConfig()", flush=True)
            print(
                "ActuatorController::initCanIdAndEthercatRelation("
                f"\"{self.ethercat_ifname}\", \"{self.actuator_config_path}\", {self.ethercat_send_frequency})",
                flush=True,
            )
            print(
                "ActuatorController::disableMotor(joint_names) -> "
                "setMotorMode(joint_names, MODE_MIT) -> enableMotor(joint_names)",
                flush=True,
            )

        print("\n[Bridge TX] protocol=EtherCAT SDK.setTargetMit(batch)", flush=True)
        print(f"ifname={self.ethercat_ifname}", flush=True)
        if self._ec_slave_port:
            m = []
            for n in names:
                sp = self._ec_slave_port.get(n, (-1, -1))
                m.append(f"{n}(slave={sp[0]},port={sp[1]})")
            print("map=" + ", ".join(m), flush=True)
        print("joint_names=" + str(names), flush=True)
        print("positions=" + str([round(x, 6) for x in pos]), flush=True)
        print("velocities=" + str([round(x, 6) for x in vel]), flush=True)
        print("torques=" + str([round(x, 6) for x in tor]), flush=True)
        print("kps=" + str([round(x, 6) for x in kp]), flush=True)
        print("kds=" + str([round(x, 6) for x in kd]), flush=True)

