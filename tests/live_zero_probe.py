#!/usr/bin/env python3
"""Live STOMP deployment check. Sends ONLY zero velocity; never moves a robot.
Run on backend VM with access to its existing environment file.
"""
import base64
import hashlib
import json
import os
import shlex
import socket
import ssl
import struct
import time
from urllib.request import Request, urlopen

HOST = '34.172.236.168'
PORT = 8081
ROBOT = 'MEDIBOT-01'


def main():
    config = {}
    with open('/etc/robot-y-ta/robot.env', encoding='utf-8') as source:
        for line in source:
            parts = shlex.split(line, comments=True)
            if parts and '=' in parts[0]:
                key, value = parts[0].split('=', 1)
                config[key] = value
    body = json.dumps({'username': config['ROBOT_AUTH_DEMO_USERNAME'],
                       'password': config['ROBOT_AUTH_DEMO_PASSWORD']}).encode()
    req = Request(f'https://{HOST}:{PORT}/api/v1/auth/login', data=body,
                  headers={'Content-Type': 'application/json'})
    with urlopen(req, timeout=8) as response:
        login = json.load(response)['data']
    assert login['user']['role'] in ('ADMIN', 'COORDINATOR')
    context = ssl.create_default_context()
    with context.wrap_socket(socket.create_connection((HOST, PORT), timeout=8),
                             server_hostname=HOST) as sock:
        sock.settimeout(5)
        key = base64.b64encode(os.urandom(16)).decode()
        sock.sendall((f'GET /ws HTTP/1.1\r\nHost: {HOST}:{PORT}\r\n'
                      f'Upgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n'
                      'Sec-WebSocket-Version: 13\r\n\r\n').encode())
        stream = sock.makefile('rb')
        assert b' 101 ' in stream.readline(), 'WebSocket upgrade rejected'
        headers = {}
        while True:
            line = stream.readline()
            if line == b'\r\n': break
            name, value = line.decode().split(':', 1)
            headers[name.lower()] = value.strip()
        expected = base64.b64encode(hashlib.sha1((key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()).decode()
        assert headers['sec-websocket-accept'] == expected

        def send(data, opcode=1):
            payload = data.encode() if isinstance(data, str) else data
            mask = os.urandom(4)
            size = len(payload)
            header = bytes([0x80 | opcode])
            if size < 126: header += bytes([0x80 | size])
            elif size < 65536: header += bytes([0x80 | 126]) + struct.pack('!H', size)
            else: header += bytes([0x80 | 127]) + struct.pack('!Q', size)
            sock.sendall(header + mask + bytes(v ^ mask[i % 4] for i, v in enumerate(payload)))

        def read_exact(size):
            data = stream.read(size)
            if len(data) != size: raise RuntimeError('WebSocket closed')
            return data

        pending = ''
        def wait(prefix):
            nonlocal pending
            while True:
                while '\x00' in pending:
                    frame, pending = pending.split('\x00', 1)
                    frame = frame.lstrip('\r\n')
                    if frame.startswith('ERROR'): raise RuntimeError('STOMP error')
                    if frame.startswith(prefix): return frame
                first, second = read_exact(2)
                opcode, size = first & 15, second & 127
                if size == 126: size = struct.unpack('!H', read_exact(2))[0]
                elif size == 127: size = struct.unpack('!Q', read_exact(8))[0]
                assert size < 1048576
                mask = read_exact(4) if second & 128 else None
                data = read_exact(size)
                if mask: data = bytes(v ^ mask[i % 4] for i, v in enumerate(data))
                if opcode == 9: send(data, 10)
                elif opcode == 8: raise RuntimeError('WebSocket closed')
                elif opcode in (0, 1): pending += data.decode()

        def request(action, data):
            send(f'SEND\ndestination:/app/robots/{ROBOT}/teleop/{action}\ncontent-type:application/json\n\n' + json.dumps(data) + '\x00')
            return json.loads(wait('MESSAGE').split('\n\n', 1)[1])

        send('CONNECT\naccept-version:1.2\nheart-beat:0,0\nAuthorization:Bearer ' + login['accessToken'] + '\n\n\x00')
        wait('CONNECTED')
        send('SUBSCRIBE\nid:deployment-check\ndestination:/user/queue/teleop-status\nack:auto\n\n\x00')
        time.sleep(.2)
        status = request('probe', {})
        print('PROBE:', json.dumps(status))
        assert status['ready'], 'Pi is not ready; no drive commands sent'
        for sequence in range(1, 4):
            ack = request('drive', {'x': 0, 'y': 0, 'yaw': 0,
                                   'sentAt': int(time.time() * 1000), 'sequence': sequence})
            assert ack['ready'], 'Zero command rejected'
            time.sleep(.1)
        print('ZERO_COMMANDS_ACCEPTED: 3; all axes exactly zero')
        send('DISCONNECT\n\n\x00')


if __name__ == '__main__':
    main()
