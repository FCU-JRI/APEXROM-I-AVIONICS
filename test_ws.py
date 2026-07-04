import asyncio
import websockets

async def test():
    try:
        async with websockets.connect("ws://localhost:8765") as ws:
            print("Connected to WebSocket!")
            for _ in range(5):
                msg = await ws.recv()
                print("Received length:", len(msg), "Data:", msg[:100], "...")
    except Exception as e:
        print("WebSocket Client Error:", e)

if __name__ == "__main__":
    asyncio.run(test())
