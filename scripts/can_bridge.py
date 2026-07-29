import can, socket, threading, sys, time
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('127.0.0.1', 9876)); sock.listen(1)
print("BRIDGE READY", flush=True)
def handle(conn):
    bus = can.interface.Bus(interface='canalystii', channel=0, bitrate=1000000)
    bus.send(can.Message(arbitration_id=0x000, data=[0x80, 0x53], is_extended_id=False))
    time.sleep(0.2)
    buf = b''
    while True:
        data = conn.recv(4096)
        if not data: break
        buf += data
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            cmd = line.decode().strip()
            if not cmd: continue
            if cmd.startswith('TX '):
                parts = cmd[3:].split()
                aid = int(parts[0], 16); dlc = int(parts[1])
                d = bytes([int(x, 16) for x in parts[2:2+dlc]])
                bus.send(can.Message(arbitration_id=aid, data=d, is_extended_id=False))
                conn.sendall(b'OK\n')
            elif cmd == 'RX':
                msg = bus.recv(timeout=0.3)
                if msg:
                    conn.sendall(f"{msg.arbitration_id:08X} {msg.dlc} {' '.join(f'{b:02X}' for b in msg.data)}\n".encode())
                else: conn.sendall(b'TIMEOUT\n')
            elif cmd == 'QUIT':
                conn.sendall(b'BYE\n'); conn.close(); bus.shutdown(); return
while True:
    c, a = sock.accept()
    threading.Thread(target=handle, args=(c,), daemon=True).start()
