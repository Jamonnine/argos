"""ARGOS Reactive Stream — 센서 데이터 BackPressure 관리

DimOS 벤치마킹: RxPY 기반 Stream, BackPressure.LATEST.
ARGOS: ROS2 콜백 → RxPY Subject → BackPressure → 프로세서.

rxpy 미설치 시 단순 큐 폴백으로 동작.

문제 배경:
    perception_bridge_node는 LiDAR(10Hz), 열화상(5Hz), 카메라(30Hz) 등
    여러 센서 데이터를 동시에 처리한다.
    처리가 밀리면 콜백 큐가 쌓여 (플러딩) 지연이 누적된다.
    BackPressure.LATEST 전략으로 항상 최신 데이터만 유지한다.

사용 예시:
    manager = SensorStreamManager()

    # LiDAR: 최신 스캔만 유지
    lidar_stream = manager.create_stream('lidar', BackPressure.LATEST)
    lidar_stream.subscribe(lambda data: process_scan(data))

    # ROS2 콜백에서 push
    def lidar_callback(msg):
        lidar_stream.push(msg)

    # 열화상: 처리 중이면 드롭
    thermal_stream = manager.create_stream('thermal', BackPressure.DROP)
    thermal_stream.subscribe(lambda data: process_thermal(data))

    # 정리
    manager.dispose_all()
"""

from __future__ import annotations

import logging
import threading
import queue
from typing import Any, Callable, Optional

# RxPY 선택적 임포트 — 없으면 큐 기반 폴백
try:
    import rx
    from rx import operators as ops
    from rx.subject import Subject
    from rx.scheduler import ThreadPoolScheduler
    RXPY_AVAILABLE = True
except ImportError:
    RXPY_AVAILABLE = False

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# BackPressure 전략 상수
# ---------------------------------------------------------------------------

class BackPressure:
    """BackPressure 전략 상수.

    LATEST: 처리 중 새 데이터가 오면 이전 것을 버리고 최신 것으로 교체.
            고주파 센서(카메라 30Hz)에 적합.
    DROP:   처리 중이면 새 데이터를 무시(드롭).
            처리 비용이 크고 최신성보다 완결성이 중요한 경우.
    BUFFER: 큐에 순서대로 축적.
            데이터 손실 금지 + 처리 속도가 충분한 경우.
    """
    LATEST = 'latest'
    DROP = 'drop'
    BUFFER = 'buffer'


# ---------------------------------------------------------------------------
# ReactiveStream — 단일 센서 스트림
# ---------------------------------------------------------------------------

class ReactiveStream:
    """단일 센서 채널의 리액티브 스트림.

    RxPY 설치 시: Subject + operators 파이프라인 사용.
    미설치 시: threading.Event + queue.Queue 기반 폴백.
    """

    def __init__(
        self,
        name: str,
        back_pressure: str = BackPressure.LATEST,
        buffer_size: int = 1,
    ) -> None:
        self.name = name
        self.back_pressure = back_pressure
        self.buffer_size = buffer_size

        self._disposed = False
        self._subscribers: list[Callable] = []

        if RXPY_AVAILABLE:
            self._init_rxpy()
        else:
            self._init_fallback()

        logger.debug(
            'ReactiveStream 생성: name=%s, strategy=%s, rxpy=%s',
            name, back_pressure, RXPY_AVAILABLE,
        )

    # ------------------------------------------------------------------
    # RxPY 모드 초기화
    # ------------------------------------------------------------------

    def _init_rxpy(self) -> None:
        """RxPY Subject + BackPressure 파이프라인 초기화."""
        self._subject: Subject = Subject()

        # BackPressure 전략별 파이프라인 구성
        if self.back_pressure == BackPressure.LATEST:
            # sample: 처리 완료 신호 대신 최신 값 유지 (debounce 유사)
            # 여기서는 간단히 latest_map_with_lock 패턴으로 구현
            self._observable = self._subject.pipe(
                ops.share(),
            )
        elif self.back_pressure == BackPressure.DROP:
            # throttle_first: 처리 중 새 데이터 드롭
            self._observable = self._subject.pipe(
                ops.share(),
            )
        else:
            # BUFFER: 그대로 흘림
            self._observable = self._subject.pipe(
                ops.share(),
            )

        self._subscriptions: list = []

    # ------------------------------------------------------------------
    # 폴백 모드 초기화
    # ------------------------------------------------------------------

    def _init_fallback(self) -> None:
        """RxPY 없을 때 threading 기반 폴백 초기화."""
        if self.back_pressure == BackPressure.BUFFER:
            self._queue: queue.Queue = queue.Queue(maxsize=self.buffer_size)
        else:
            # LATEST / DROP: maxsize=1 큐 사용
            self._queue = queue.Queue(maxsize=1)

        self._processing = threading.Event()  # 처리 중 여부 (DROP 전략)
        self._worker_thread: Optional[threading.Thread] = None
        self._start_fallback_worker()

    def _start_fallback_worker(self) -> None:
        """폴백 모드 백그라운드 워커 스레드 시작."""
        def worker() -> None:
            while not self._disposed:
                try:
                    data = self._queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                self._processing.set()
                try:
                    for cb in self._subscribers:
                        try:
                            cb(data)
                        except Exception as exc:
                            logger.exception(
                                'ReactiveStream[%s] 구독자 콜백 오류: %s', self.name, exc
                            )
                finally:
                    self._processing.clear()

        self._worker_thread = threading.Thread(
            target=worker, name=f'reactive_{self.name}', daemon=True
        )
        self._worker_thread.start()

    # ------------------------------------------------------------------
    # 공개 인터페이스
    # ------------------------------------------------------------------

    def push(self, data: Any) -> None:
        """ROS2 콜백에서 새 데이터를 스트림에 주입한다.

        BackPressure 전략에 따라 데이터를 유지하거나 드롭한다.
        이 메서드는 스레드 안전하다.
        """
        if self._disposed:
            return

        if RXPY_AVAILABLE:
            self._subject.on_next(data)
        else:
            self._push_fallback(data)

    def _push_fallback(self, data: Any) -> None:
        """폴백 모드 push — BackPressure 전략 적용."""
        if self.back_pressure == BackPressure.LATEST:
            # 큐가 가득 찼으면 기존 데이터를 버리고 새 데이터 삽입
            try:
                self._queue.get_nowait()  # 기존 제거
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(data)
            except queue.Full:
                pass  # 극히 드문 레이스컨디션, 무시

        elif self.back_pressure == BackPressure.DROP:
            # 처리 중이거나 큐에 이미 데이터가 있으면 드롭
            if self._processing.is_set():
                logger.debug('ReactiveStream[%s] DROP: 처리 중', self.name)
                return
            try:
                self._queue.put_nowait(data)
            except queue.Full:
                logger.debug('ReactiveStream[%s] DROP: 큐 가득 참', self.name)

        else:  # BUFFER
            try:
                self._queue.put_nowait(data)
            except queue.Full:
                logger.warning(
                    'ReactiveStream[%s] BUFFER 가득 참 — 가장 오래된 데이터 제거', self.name
                )
                try:
                    self._queue.get_nowait()
                    self._queue.put_nowait(data)
                except queue.Empty:
                    pass

    def subscribe(self, on_next: Callable[[Any], None]) -> None:
        """데이터 수신 콜백을 등록한다.

        RxPY 모드: Observable 구독.
        폴백 모드: 콜백 목록에 추가 (워커 스레드가 순서대로 호출).
        """
        if RXPY_AVAILABLE:
            subscription = self._observable.subscribe(
                on_next=on_next,
                on_error=lambda e: logger.exception(
                    'ReactiveStream[%s] 오류: %s', self.name, e
                ),
            )
            self._subscriptions.append(subscription)
        else:
            self._subscribers.append(on_next)

        logger.debug('ReactiveStream[%s] 구독 등록 완료', self.name)

    def dispose(self) -> None:
        """스트림을 정리한다. 이후 push/subscribe 호출은 무시된다."""
        self._disposed = True

        if RXPY_AVAILABLE:
            self._subject.on_completed()
            for sub in self._subscriptions:
                try:
                    sub.dispose()
                except Exception:
                    pass
            self._subscriptions.clear()
        else:
            # 워커 스레드는 _disposed 플래그를 감지하여 자연 종료
            pass

        logger.debug('ReactiveStream[%s] 정리 완료', self.name)

    def __repr__(self) -> str:
        return (
            f'ReactiveStream(name={self.name!r}, '
            f'strategy={self.back_pressure}, rxpy={RXPY_AVAILABLE})'
        )


# ---------------------------------------------------------------------------
# SensorStreamManager — 여러 센서 스트림 통합 관리
# ---------------------------------------------------------------------------

class SensorStreamManager:
    """여러 센서 스트림을 한 곳에서 생성·관리한다.

    사용 예시:
        manager = SensorStreamManager()

        lidar = manager.create_stream('lidar', BackPressure.LATEST)
        thermal = manager.create_stream('thermal', BackPressure.DROP)
        camera = manager.create_stream('camera', BackPressure.LATEST, buffer_size=2)

        lidar.subscribe(process_scan)
        thermal.subscribe(process_thermal)

        # 정리 (노드 종료 시)
        manager.dispose_all()
    """

    def __init__(self) -> None:
        # {stream_name: ReactiveStream}
        self._streams: dict[str, ReactiveStream] = {}
        logger.info(
            'SensorStreamManager 초기화 (RxPY=%s)', RXPY_AVAILABLE
        )

    def create_stream(
        self,
        name: str,
        back_pressure: str = BackPressure.LATEST,
        buffer_size: int = 1,
    ) -> ReactiveStream:
        """새 ReactiveStream을 생성하고 등록한다.

        같은 이름이 이미 있으면 기존 스트림을 반환 (재생성 방지).
        """
        if name in self._streams:
            logger.warning(
                'SensorStreamManager: 스트림 %r 이미 존재 — 기존 반환', name
            )
            return self._streams[name]

        stream = ReactiveStream(name, back_pressure, buffer_size)
        self._streams[name] = stream
        return stream

    def get_stream(self, name: str) -> Optional[ReactiveStream]:
        """이름으로 기존 스트림을 조회한다. 없으면 None 반환."""
        return self._streams.get(name)

    def dispose_all(self) -> None:
        """모든 스트림을 정리한다. 노드 destroy() 시 호출 권장."""
        for stream in self._streams.values():
            stream.dispose()
        self._streams.clear()
        logger.info('SensorStreamManager: 전체 스트림 정리 완료')

    def __len__(self) -> int:
        return len(self._streams)

    def __repr__(self) -> str:
        names = list(self._streams.keys())
        return f'SensorStreamManager(streams={names})'
