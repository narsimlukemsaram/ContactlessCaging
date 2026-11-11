"""Decode the historical PhaseSpace C++ UDP tracker wire format."""
import math
import struct

TRACKER_SIZE = 32 + 32 + 4 * 20  # fixed 32-byte name, 8 floats, four 20-byte markers


def decode_packet(data: bytes):
    if len(data) < 4:
        raise ValueError('Packet too short')
    (count,) = struct.unpack_from('<i',data,0)
    if not 0 <= count <= 12:
        raise ValueError('Unexpected tracker count')
    if len(data) < 4 + count * TRACKER_SIZE:
        raise ValueError('Truncated tracker data')
    offset = 4
    out = []
    for _ in range(count):
        rawname=data[offset:offset+32]
        name=rawname.split(b'\0',1)[0].decode('ascii','ignore').strip()
        values=struct.unpack_from('<8f',data,offset+32)
        x,y,z = values[:3]
        heading_y = values[7]
        if name and all(math.isfinite(v) for v in (x,y,z,heading_y)):
            out.append((name,x,y,z,heading_y))
        offset += TRACKER_SIZE
    return out

