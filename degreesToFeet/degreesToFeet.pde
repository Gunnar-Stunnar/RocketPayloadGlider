void setup() {
  float radToDegrees = 180/3.14159;
  //subtract destination/home lat long to get the difference 
  //lat1, lon1 = destination/home
  float lat1 = 0;
  float lon1 = 0;
  float lat2 = 1.01;
  float lon2 = 1.8;
  
  float R = 20925524.9; // feet
  //convert to to radians
  float Lat1 = lat1/radToDegrees;
  float Lat2 = lat2/radToDegrees;
  float Lon1 = lon1/radToDegrees;
  float Lon2 = lon2/radToDegrees;
  //find x y values in feet
  float DiffLat = (lat2-lat1)/radToDegrees;
  float DiffLon = (lon2-lon1)/radToDegrees;
  //calc difference in x (lat)
  float aX = sin(Lat2/2) * sin(Lat2/2);
  double x = R * 2 * atan2(sqrt(aX), sqrt(1-aX));
  //calc difference in y (long)
  float aY = sin(Lon2/2) * sin(Lon2/2);
  double y = R * 2 * atan2(sqrt(aY), sqrt(1-aY));
  
  //actual distance in feet
  float a = sin(DiffLat/2) * sin(DiffLat/2) + cos(Lat1) * cos(Lat2) * sin(DiffLon/2) * sin(DiffLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  double d = R * c;
  
  System.out.println("Approximate distance between HR and FC:");
  System.out.println("x="+x);
  System.out.println("y="+y);
  System.out.println(sqrt((float)(x*x+y*y)));
  System.out.println(d);

}