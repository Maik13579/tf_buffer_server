# Copyright 2026 Maik Knof
# SPDX-License-Identifier: Apache-2.0
"""Checked server and transport failures."""
from tf_buffer_server_interfaces.msg import BufferStatus


class ClientError(RuntimeError):
    """Failure with a kind and optional original BufferStatus."""

    def __init__(self, message, *, kind='server', status=None):
        super().__init__(message)
        self.kind = kind
        self.status = status


def checked(response):
    if response.status.code != BufferStatus.OK:
        raise ClientError(response.status.message, status=response.status)
    return response


def availability(response):
    if response.status.code in (BufferStatus.LOOKUP_ERROR, BufferStatus.CONNECTIVITY_ERROR,
                               BufferStatus.EXTRAPOLATION_ERROR, BufferStatus.TIMEOUT):
        return False
    return checked(response).can_transform
