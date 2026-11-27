"""Immediate channel5 datagram; payload and measurement stamps unchanged."""
from unicast_wire import encode

def send_status(sock,peer,port,sequence,stamp_ns,raw):
    next_sequence=sequence+1
    sock.sendto(encode(5,next_sequence,stamp_ns,raw),(peer,port))
    return next_sequence
