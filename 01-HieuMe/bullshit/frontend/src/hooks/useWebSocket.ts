import { useState, useEffect, useCallback, useRef } from 'react';

export function useWebSocket(url: string) {
  const [connected, setConnected] = useState(false);
  const [lastMessage, setLastMessage] = useState<any>(null);
  const ws = useRef<WebSocket | null>(null);
  const reconnectAttempts = useRef(0);
  const maxReconnectDelay = 10000;

  const connect = useCallback(() => {
    try {
      ws.current = new WebSocket(url);
      
      ws.current.onopen = () => {
        setConnected(true);
        reconnectAttempts.current = 0;
      };

      ws.current.onclose = () => {
        setConnected(false);
        const delay = Math.min(1000 * Math.pow(2, reconnectAttempts.current), maxReconnectDelay);
        reconnectAttempts.current++;
        setTimeout(connect, delay);
      };

      ws.current.onerror = (error) => {
        console.error('WebSocket Error:', error);
        ws.current?.close();
      };

      ws.current.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          setLastMessage(data);
        } catch (e) {
          console.error('WebSocket Parse Error:', e);
        }
      };
    } catch (e) {
      console.error('WebSocket Connect Error:', e);
    }
  }, [url]);

  useEffect(() => {
    connect();
    return () => {
      ws.current?.close();
    };
  }, [connect]);

  const sendMessage = useCallback((data: any) => {
    if (ws.current && ws.current.readyState === WebSocket.OPEN) {
      ws.current.send(JSON.stringify(data));
    }
  }, []);

  return { connected, lastMessage, sendMessage };
}
