import asyncio
import websockets
import json
import sys

async def test():
    uri = 'ws://localhost:8000/ws'
    print("Connecting to WebSocket at ws://localhost:8000/ws ...")
    try:
        async with websockets.connect(uri) as ws:
            messages = {}
            for _ in range(50):
                try:
                    raw = await asyncio.wait_for(ws.recv(), timeout=2.0)
                    data = json.loads(raw)
                except asyncio.TimeoutError:
                    break
                msg_type = data.get('type')
                if msg_type and msg_type not in messages:
                    messages[msg_type] = True
                    sample = {k: v for k, v in data.items() if k not in ('data', 'points')}
                    if msg_type == 'map_update':
                        sample['data'] = f"[{len(data.get('data',[]))} cells]"
                    elif msg_type in ('lidar_scan', 'trajectory'):
                        sample['points'] = f"[{len(data.get('points',[]))} pts]"
                    print(f"  Received {msg_type}: {json.dumps(sample, default=str)[:100]}")
                if len(messages) >= 3:
                    break

            print(f"\nReceived {len(messages)} message types from backend.")
            
            # Test set_mode
            await ws.send(json.dumps({'type': 'set_mode', 'mode': 'AUTONOMOUS'}))
            print("Sent set_mode: AUTONOMOUS")

            # Test set_goal
            await ws.send(json.dumps({'type': 'set_goal', 'x': 1.5, 'y': 0.5}))
            print("Sent set_goal: (1.5, 0.5)")

            # Test stop
            await ws.send(json.dumps({'type': 'stop'}))
            print("Sent stop")

            print("\n=== WebSocket Test PASSED! ===")
            return True
    except Exception as e:
        print(f"WebSocket Test connection error: {e}")
        return False

if __name__ == '__main__':
    ok = asyncio.run(test())
    sys.exit(0 if ok else 1)
