import asyncio
import websockets
import json

async def test():
    async with websockets.connect("ws://localhost:8765") as websocket:
        print("Connected!")
        # Send command to switch to CAL_GYRO (state 2)
        await websocket.send(json.dumps({"type": "cmd", "action": "setState", "stateId": 2}))
        
        for _ in range(15):
            msg = await websocket.recv()
            data = json.loads(msg)
            for d in data["batch"]:
                if d["type"] == "LOG":
                    print("LOG:", d["ts"], d["data"]["msg"])

asyncio.run(test())
