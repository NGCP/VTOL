from math import sqrt, tan, atan2

class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y
    
    def __repr__(self):
        coordinates = (self.x, self.y)
        return str(coordinates)


def convert(POI, alt, p0 = Point(0,0), originLatLon = Point(0,0)):

    LAT_METERS_TO_DEGREES = 110574.61087757687
    LON_METERS_TO_DEGREES = 111302.61697430261
    ALT_PIXEL_X = 2718.65906861
    ALT_PIXEL_Y = 2157.06893551

    delta_X = POI.x - p0.x 
    delta_Y = POI.y - p0.y

    while delta_X > 1640:
        p0.x+= 1640
        delta_X = POI.x - p0.x
        xAngle = atan2(1640, ALT_PIXEL_X)
        xMeters = alt * tan(xAngle)
        originLatLon.y += (xMeters / LON_METERS_TO_DEGREES)
    
    while delta_X < -1640:
        p0.x -= 1640
        delta_X = POI.x - p0.x
        xAngle = atan2(1640, ALT_PIXEL_X)
        xMeters = alt * tan(xAngle)
        originLatLon.y -= (xMeters / LON_METERS_TO_DEGREES)

    while delta_Y > 1232:
        p0.y += 1232
        delta_Y = POI.y - p0.y
        yAngle = atan2(1232, ALT_PIXEL_Y)
        yMeters = alt * tan(yAngle)
        originLatLon.x += (yMeters / LAT_METERS_TO_DEGREES)
        
    while delta_Y < -1232:
        p0.y -= 1232
        delta_Y = POI.y - p0.y
        yAngle = atan2(1232, ALT_PIXEL_Y)
        yMeters = alt * tan(yAngle)
        originLatLon.x -= (yMeters / LAT_METERS_TO_DEGREES)
    
    xAngle = atan2(delta_X, ALT_PIXEL_X)
    yAngle = atan2(delta_Y, ALT_PIXEL_Y)

    xMeters = alt * tan(xAngle)
    yMeters = alt * tan(yAngle)

    lat = (yMeters / LAT_METERS_TO_DEGREES ) + originLatLon.x
    lon = (xMeters / LON_METERS_TO_DEGREES) + originLatLon.y

    return Point(lat, lon)

##################################################################################################
if __name__ == '__main__':
    POI = Point(9093, 9731)
    latLonOrigin = Point(35.299504, -120.663127)
    origin = Point(1313, 9409)
    print('Origin: ',latLonOrigin)
    print('POI: ',convert(POI, 90, origin, latLonOrigin))
