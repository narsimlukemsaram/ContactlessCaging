import struct
from contactlesscaging_control.legacy_packet import decode_packet, TRACKER_SIZE

def test_decode_packet():
    name=b'AcoustoBot1' + bytes(32-len('AcoustoBot1'))
    state=struct.pack('<8f',.2,.1,.05,0,0,0,1,0)+bytes(4*20)
    packet=struct.pack('<i',1)+name+state
    assert len(name+state)==TRACKER_SIZE
    out=decode_packet(packet)
    assert len(out)==1 and out[0][0]=='AcoustoBot1' and abs(out[0][1]-.2) < 1e-6

def test_bad_packet():
    for payload in (b'',struct.pack('<i',13),struct.pack('<i',1)):
        try: decode_packet(payload)
        except ValueError: pass
        else: raise AssertionError('Expected malformed packet rejection')
