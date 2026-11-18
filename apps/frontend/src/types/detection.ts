export interface Detection {
  className: string;
  confidence: number;
  bbox: [number, number, number, number];
  distance: number;
  azimuth_deg?: number;
  azimuthDeg?: number;
}
