# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Per-request lifetime, real-time deadlines and local cancellation."""
from threading import Event, RLock
from rclpy.clock import Clock, ClockType
from rclpy.task import Future
from .errors import ClientError


class ResultFuture(Future):
    """An rclpy future with local cancellation and a bounded blocking helper."""

    def __init__(self, budget):
        super().__init__()
        self._event = Event()
        self._budget = budget
        self._expire = None
        self._cancel_request = None
        self.add_done_callback(lambda _: self._event.set())

    def cancel(self):
        if self._cancel_request is not None:
            self._cancel_request()

    def wait(self):
        if not self.done() and not self._event.wait(self._budget):
            if self._expire is not None:
                self._expire()
        if self.cancelled():
            raise ClientError('Request cancelled locally', kind='cancelled')
        return self.result()


class Pending:
    def __init__(self, owner, client, budget, decode):
        self.owner = owner
        self.client = client
        self.decode = decode
        self.future = ResultFuture(budget)
        self.future._expire = self.expire
        self.future._cancel_request = lambda: self.finish(cancelled=True)
        self.lock = RLock()
        self.done = False
        self.raw = None
        self.timer = None

    def start(self, request):
        try:
            with self.lock:
                if self.done:
                    return
                if not self.client.service_is_ready():
                    raise ClientError(f'Service unavailable: {self.client.srv_name}',
                                      kind='unavailable')
                self.raw = self.client.call_async(request)
                self.timer = self.owner._node.create_timer(
                    self.future._budget, self.expire,
                    callback_group=self.owner._group,
                    clock=Clock(clock_type=ClockType.STEADY_TIME))
                self.raw.add_done_callback(self.response)
        except Exception as error:
            self.finish(error=error)

    def response(self, raw):
        try:
            value = self.decode(raw.result())
        except Exception as error:
            self.finish(error=error)
        else:
            self.finish(value=value)

    def expire(self):
        self.finish(error=ClientError('Timed out waiting for service response',
                                     kind='response_timeout'))

    def finish(self, *, value=None, error=None, cancelled=False):
        with self.lock:
            if self.done:
                return
            self.done = True
            raw, timer = self.raw, self.timer
            self.raw = self.timer = None
        if timer is not None:
            timer.cancel()
            self.owner._node.destroy_timer(timer)
        if raw is not None:
            self.client.remove_pending_request(raw)
        self.owner._forget(self)
        if cancelled:
            Future.cancel(self.future)
        elif error is not None:
            self.future.set_exception(error)
        else:
            self.future.set_result(value)
        self.future._expire = self.future._cancel_request = None
