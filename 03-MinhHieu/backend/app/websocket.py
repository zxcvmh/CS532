"""
Backward compatibility facade for WebSocketHub.
Re-exports WebSocketManager pointing to app.streaming.websocket_hub.WebSocketHub.
"""
from app.streaming.websocket_hub import WebSocketHub

# Backward-compatibility alias
WebSocketManager = WebSocketHub

__all__ = ["WebSocketManager", "WebSocketHub"]
