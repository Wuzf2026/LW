# Hesai JT128 point cloud data
HesaiPoint[] points    # Array of points
uint64 timestamp       # Timestamp in microseconds

# Individual point structure
MSG: lawnwomer/HesaiPoint
float32 x              # X coordinate in meters
float32 y              # Y coordinate in meters
float32 z              # Z coordinate in meters
uint16 intensity       # Intensity (0-255)