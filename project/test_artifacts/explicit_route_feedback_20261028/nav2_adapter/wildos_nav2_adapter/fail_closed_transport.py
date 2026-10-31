"""Fail closed when BeamNGpy loses framing on a timed-out socket read.

BeamNGpy's default ``PrefixedLengthSocket._recv_exactly`` reconnects after
any socket error, including ``socket.timeout``, then continues reading the
old response on the new stream. That stream has no request/response context.
Close it and propagate the failure so the owning node can change health state
instead of silently treating a new TCP connection as the old transaction.
"""


def install_fail_closed_reads(socket_class):
    current = socket_class._recv_exactly
    if getattr(current, '_wildos_fail_closed', False):
        return

    def recv_exactly(self, length):
        remaining = int(length)
        chunks = []
        while remaining > 0:
            try:
                chunk = self.skt.recv(min(65536, remaining))
            except Exception as exc:
                try:
                    self.close()
                except Exception:
                    pass
                raise exc
            if not chunk:
                try:
                    self.close()
                except Exception:
                    pass
                raise ConnectionError('BeamNG closed a partial response stream')
            chunks.append(chunk)
            remaining -= len(chunk)
        return b''.join(chunks)

    recv_exactly._wildos_fail_closed = True
    socket_class._recv_exactly = recv_exactly
