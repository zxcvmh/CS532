export type EventLevel = 'INFO' | 'WARNING' | 'ERROR' | 'SUCCESS';

export interface EventLogEntry {
  timestamp: string;
  level: EventLevel;
  message: string;
}
