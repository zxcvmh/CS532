import { useState, useEffect, useCallback, useRef } from 'react';

export function useWebSocket(url: string) {
  const [connected, setConnected] = useState(false);
  const [lastMessage, setLastMessage] = useState<any>(null);
  const [rtt, setRtt] = useState<number | null>(null);
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
        setRtt(null);
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
          // UIT Tut 3 Heartbeat pong response
          if (data.type === 'pong' && typeof data.t === 'number') {
            const roundTrip = Math.max(1, Math.round(performance.now() - data.t));
            setRtt(roundTrip);
            return;
          }
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

  // UIT Tut 3 Heartbeat ping loop (1 Hz) to measure network RTT latency
  useEffect(() => {
    if (!connected) return;
    const pingTimer = setInterval(() => {
      if (ws.current && ws.current.readyState === WebSocket.OPEN) {
        ws.current.send(JSON.stringify({ type: 'ping', t: performance.now() }));
      }
    }, 1000);
    return () => clearInterval(pingTimer);
  }, [connected]);

  const sendMessage = useCallback((data: any) => {
    if (ws.current && ws.current.readyState === WebSocket.OPEN) {
      ws.current.send(JSON.stringify(data));
    }
  }, []);

  return { connected, lastMessage, sendMessage, rtt };
}
