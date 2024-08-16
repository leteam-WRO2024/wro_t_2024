import numpy as np
LEFT_RIGHT_REGIONS_WIDTH = 100

FRAME_WIDTH  =   480
FRAME_HEIGHT =   300
FRAME_CENTER =   FRAME_WIDTH // 2
CROP_HEIGHT  =   140
CROP_WIDTH   =   80
CROP_LEFT    =   FRAME_WIDTH  - LEFT_RIGHT_REGIONS_WIDTH
CROP_RIGHT   =   LEFT_RIGHT_REGIONS_WIDTH


COLOR_RANGES = {
    "green":    (np.array([40, 40, 20], np.uint8), np.array([95, 255, 205], np.uint8)),
    "red":      (np.array([160, 110, 60], np.uint8), np.array([1757, 255, 255], np.uint8)),
    "orange":   (np.array([0, 30, 50], np.uint8), np.array([24, 255, 255], np.uint8)),
    "blue":     (np.array([100, 70, 51], np.uint8), np.array([130, 255, 255], np.uint8)),
    "black":    (np.array([0, 0, 0], np.uint8), np.array([180, 255, 50], np.uint8))
}

CROP_PARAMS = {
    "orange":       (0, 0, 0),
    "blue":         (0, 0, 0),
    "red":          (CROP_HEIGHT, FRAME_WIDTH - CROP_WIDTH, CROP_WIDTH),
    "green":        (CROP_HEIGHT, FRAME_WIDTH - CROP_WIDTH, CROP_WIDTH),
    "black":        (CROP_HEIGHT, 0, 0),
    "black_right":  (0, FRAME_WIDTH, CROP_LEFT),
    "black_left":   (0, CROP_RIGHT, 0),
}

