"""行形式の入力を最新値として保持する。書込み・自動再接続はしない。"""
import threading
import time


class LineInput:
    def __init__(self, parser, max_age, max_line=65536):
        self.parser, self.max_age, self.max_line = parser, max_age, max_line
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.latest = self.error = self.thread = self.closer = None

    def start(self, read, close, timeout):
        if self.thread is not None:
            raise RuntimeError("入力は接続済みです")
        self.closer = close
        self.stop.clear()
        self.ready.clear()
        self.latest = self.error = None

        def run():
            buffer = b""
            try:
                while not self.stop.is_set():
                    chunk = read()
                    if chunk is None:  # 通信タイムアウト。部分行は次の受信まで保持する。
                        continue
                    if not chunk:
                        raise RuntimeError("入力接続が切れました")
                    buffer += chunk
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        if len(line) > self.max_line:
                            raise ValueError("入力行が長すぎます")
                        received = time.monotonic()
                        frame = self.parser(line.decode("ascii"), received)
                        with self.lock:
                            self.latest = frame
                        self.ready.set()
                    if len(buffer) > self.max_line:
                        raise ValueError("未完の入力行が長すぎます")
            except Exception as exc:
                if not self.stop.is_set():
                    with self.lock:
                        self.error, self.latest = exc, None
                    self.ready.set()

        self.thread = threading.Thread(target=run, name="yubi-input", daemon=True)
        self.thread.start()
        try:
            if not self.ready.wait(timeout):
                raise TimeoutError("入力データを受信できません")
            self.snapshot()
        except BaseException:
            self.close()
            raise

    def snapshot(self):
        with self.lock:
            if self.error:
                raise RuntimeError(f"入力エラー: {self.error}") from self.error
            if self.latest is None:
                raise RuntimeError("入力未接続です")
            frame = dict(self.latest)
        if not 0 <= time.monotonic() - frame["received"] <= self.max_age:
            raise TimeoutError("入力が古いため停止します")
        return frame

    def close(self):
        self.stop.set()
        try:
            if self.closer:
                self.closer()
        finally:
            if self.thread:
                self.thread.join(timeout=2)
                if self.thread.is_alive():
                    raise RuntimeError("入力読取りスレッドを終了できません")
            self.thread = self.closer = self.latest = None
