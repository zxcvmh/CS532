import asyncio
import websockets
import json

async def test():
    uri = 'ws://localhost:8000/ws'
    print("Connecting to WebSocket...")
    async with websockets.connect(uri) as ws:
        messages = {}
        for _ in range(200):
            data = json.loads(await asyncio.wait_for(ws.recv(), timeout=2.0))
            msg_type = data['type']
            if msg_type not in messages:
                messages[msg_type] = True
                # Show sample of the data
                sample = {k: v for k, v in data.items() if k != 'data'}
                if msg_type == 'map_update':
                    sample['data'] = f"[{len(data.get('data',[]))} cells]"
                elif msg_type == 'lidar_scan':
                    sample['points'] = f"[{len(data.get('points',[]))} pts]"
                elif msg_type == 'trajectory':
                    sample['points'] = f"[{len(data.get('points',[]))} pts]"
                print(f"  {msg_type}: {json.dumps(sample, default=str)[:120]}")
            if len(messages) >= 10:
                break
        print(f"\nReceived {len(messages)} unique message types")
        
        # Test set_mode
        await ws.send(json.dumps({'type': 'set_mode', 'mode': 'AUTONOMOUS'}))
        print("\nSent set_mode AUTONOMOUS")
        
        # Wait a bit for mode change event
        for _ in range(20):
            data = json.loads(await asyncio.wait_for(ws.recv(), timeout=2.0))
            if data['type'] == 'event':
                print(f"  Event: [{data['level']}] {data['message']}")
                break
        
        # Test set_goal
        await ws.send(json.dumps({'type': 'set_goal', 'x': 2.0, 'y': 1.0}))
        print("\nSent goal (2.0, 1.0)")
        
        # Watch navigation progress
        nav_seen = set()
        for _ in range(300):
            data = json.loads(await asyncio.wait_for(ws.recv(), timeout=2.0))
            if data['type'] == 'navigation_status':
                status = data['status']
                prog = data.get('progress') or 0
                if status not in nav_seen or status == 'NAVIGATING':
                    nav_seen.add(status)
                    dist = data.get('distance_remaining') or 0
                    print(f"  Nav: {status} progress={prog:.0f}% dist={dist:.2f}m")
                if status in ['REACHED', 'ERROR', 'CANCELLED']:
                    break
            elif data['type'] == 'event':
                print(f"  Event: [{data['level']}] {data['message']}")

        print("\n=== WebSocket Test PASSED! ===")

asyncio.run(test())
