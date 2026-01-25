# RTK UM982 positioning data
float64 latitude       # Latitude in degrees
float64 longitude      # Longitude in degrees
float64 altitude       # Altitude in meters
float32 hdop           # Horizontal dilution of precision
int32 satellite_count  # Number of satellites
int32 fix_type         # 0:no fix, 1:2D, 2:3D, 3:RTK fixed, 4:RTK float
uint64 timestamp       # Timestamp in milliseconds