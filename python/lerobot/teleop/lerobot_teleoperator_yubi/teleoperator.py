"""生入力は標準processorで関節目標へ変換する。SDKには依存しない。"""
from contextlib import ExitStack
from lerobot.teleoperators import Teleoperator
from .config import YubiTeleoperatorConfig, SIDES
from .quest import QuestInput
from .gripper import GripperInput


class YubiTeleoperator(Teleoperator):
    config_class = YubiTeleoperatorConfig
    name = "yubi"

    def __init__(self, config):
        config.validate()
        super().__init__(config)
        self.config = config
        self.quest = QuestInput(config)
        self.grippers = {side: GripperInput(config.arms[side], input_timeout_s=config.input_timeout_s,
                                          connect_timeout_s=config.connect_timeout_s) for side in SIDES}
        self._connected = False

    @property
    def action_features(self):
        poses = list(SIDES) + (["head"] if self.config.neck_mode == "follow" else [])
        keys = [f"{p}.{k}{i}" for p in poses for k, n in (("p", 3), ("q", 4)) for i in range(n)]
        keys += [f"{side}.{key}" for side in SIDES
                 for key in ("clutch", "releases", "encoder_received", "encoder_rad", "opening")]
        return dict.fromkeys(keys + ["received", "device_time_ns"], float)

    @property
    def feedback_features(self):
        return {}

    @property
    def is_connected(self):
        return self._connected

    @property
    def is_calibrated(self):
        return self.config.hardware_verified

    def connect(self, calibrate=True):
        if self._connected:
            raise RuntimeError("YUBIは接続済みです")
        if not self.is_calibrated:
            raise RuntimeError("YUBIの取付け・開口校正を確認後、hardware_verifiedを設定してください")
        try:
            self.quest.connect()
            for source in self.grippers.values():
                source.connect()
            self._connected = True
        except BaseException:
            self.disconnect()
            raise

    def get_action(self):
        if not self._connected:
            raise RuntimeError("YUBIは未接続です")
        result = self.quest.snapshot()
        for side, source in self.grippers.items():
            finger = source.snapshot()
            result.update({f"{side}.encoder_received": finger["received"],
                           f"{side}.encoder_rad": finger["encoder_rad"], f"{side}.opening": finger["opening"]})
        return result

    def calibrate(self):
        raise NotImplementedError("確認デモで開口値と座標系を確認し、設定へ記入してください")

    def configure(self):
        pass

    def send_feedback(self, feedback):
        if feedback:
            raise NotImplementedError("力覚フィードバックは未対応です")

    def disconnect(self):
        self._connected = False
        # 途中で接続に失敗した場合も、開いた入力をすべて閉じる。
        with ExitStack() as cleanup:
            for source in [self.quest, *self.grippers.values()]:
                cleanup.callback(source.close)
