export interface MapData {
  resolution: number;
  width: number;
  height: number;
  originX: number;
  originY: number;
  data: number[];
}

export interface LidarPoint {
  x: number;
  y: number;
}

export interface MapTransform {
  scale: number;
  offsetX: number;
  offsetY: number;
}
