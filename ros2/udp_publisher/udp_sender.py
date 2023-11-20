# На основе https://docs.python.org/3/library/asyncio-protocol.html

import asyncio


async def send_task(message, transport):
    while True:
        print('Send:', message)
        transport.sendto(message)
        await asyncio.sleep(1)


class EchoClientProtocol:
    def __init__(self, message, on_con_lost):
        self.message = message
        self.on_con_lost = on_con_lost
        self.transport = None

    def connection_made(self, transport):
        self.transport = transport
        asyncio.create_task(send_task(self.message.encode(), transport))

    def datagram_received(self, data, addr):
        print("Received:", data.decode())

    def error_received(self, exc):
        print('Error received:', exc)

    def connection_lost(self, exc):
        print("Connection closed")
        self.on_con_lost.set_result(True)


async def main():
    # Get a reference to the event loop as we plan to use
    # low-level APIs.
    loop = asyncio.get_running_loop()

    on_con_lost = loop.create_future()
    message = "Hello World!"

    transport, protocol = await loop.create_datagram_endpoint(
        lambda: EchoClientProtocol(message, on_con_lost),
        remote_addr=('127.0.0.1', 5005))

    try:
        await on_con_lost
    finally:
        transport.close()


asyncio.run(main())
