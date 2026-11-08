export interface MapData {
  resolution: number;
  width: number;
  height: number;
  originX: number;
  originY: number;
  data: number[] | Int8Array | ArrayLike<number>;
  version?: number;
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
