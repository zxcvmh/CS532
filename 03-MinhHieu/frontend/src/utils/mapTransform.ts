import { MapData, MapTransform } from '../types/map';

export function mapToScreen(
  mapX: number,
  mapY: number,
  transform: MapTransform,
  canvasWidth: number,
  canvasHeight: number,
  mapData: MapData | null
): { x: number; y: number } {
  // mapData origin is usually bottom-left or center. Let's assume standard ROS coordinate (0,0 center).
  // Y axis in canvas is inverted.
  const scaledX = mapX * transform.scale;
  const scaledY = -mapY * transform.scale; // Invert Y

  return {
    x: canvasWidth / 2 + scaledX + transform.offsetX,
    y: canvasHeight / 2 + scaledY + transform.offsetY,
  };
}

export function screenToMap(
  screenX: number,
  screenY: number,
  transform: MapTransform,
  canvasWidth: number,
  canvasHeight: number,
  mapData: MapData | null
): { x: number; y: number } {
  const scaledX = screenX - canvasWidth / 2 - transform.offsetX;
  const scaledY = screenY - canvasHeight / 2 - transform.offsetY;

  return {
    x: scaledX / transform.scale,
    y: -scaledY / transform.scale,
  };
}
