export type EventLevel = 'INFO' | 'WARNING' | 'ERROR' | 'SUCCESS';

export interface EventLogEntry {
  timestamp: string | number;
  level: EventLevel;
  message: string;
}
