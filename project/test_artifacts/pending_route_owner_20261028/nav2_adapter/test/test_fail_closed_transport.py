import unittest

from wildos_nav2_adapter.fail_closed_transport import install_fail_closed_reads


class FakeSocket:
    def __init__(self, chunks=(), error=None):
        self.chunks = list(chunks)
        self.error = error
        self.closed = False
        self.reconnect_calls = 0

    def recv(self, _length):
        if self.error is not None:
            raise self.error
        return self.chunks.pop(0) if self.chunks else b''


class FakePrefixedSocket:
    def __init__(self, sock):
        self.skt = sock
        self.recv_buffer = []

    def close(self):
        self.skt.closed = True

    def reconnect(self):
        self.skt.reconnect_calls += 1

    def _recv_exactly(self, _length):
        raise AssertionError('test method should be replaced')


class FailClosedTransportTests(unittest.TestCase):
    def test_reads_complete_a_length_prefixed_payload_in_chunks(self):
        socket_class = type('Socket', (FakePrefixedSocket,), {})
        install_fail_closed_reads(socket_class)
        stream = socket_class(FakeSocket([b'ab', b'cd']))
        self.assertEqual(stream._recv_exactly(4), b'abcd')
        self.assertFalse(stream.skt.closed)

    def test_timeout_closes_stream_and_never_reconnects_mid_transaction(self):
        socket_class = type('Socket', (FakePrefixedSocket,), {})
        install_fail_closed_reads(socket_class)
        stream = socket_class(FakeSocket(error=TimeoutError('read timed out')))
        with self.assertRaisesRegex(TimeoutError, 'read timed out'):
            stream._recv_exactly(4)
        self.assertTrue(stream.skt.closed)
        self.assertEqual(stream.skt.reconnect_calls, 0)

    def test_peer_close_during_partial_response_is_terminal(self):
        socket_class = type('Socket', (FakePrefixedSocket,), {})
        install_fail_closed_reads(socket_class)
        stream = socket_class(FakeSocket([b'ab', b'']))
        with self.assertRaisesRegex(ConnectionError, 'partial response'):
            stream._recv_exactly(4)
        self.assertTrue(stream.skt.closed)
        self.assertEqual(stream.skt.reconnect_calls, 0)


if __name__ == '__main__':
    unittest.main()
