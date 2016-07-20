


float a = 1.1556;
float b = 0.5 + (a/4); 
float c = (float)sqrt(a*a-b*b);
float points[56];

float target[2];
float pos[2];    ///WE NEED TO SET THESE-----------------------------------------------------------------------------
float fang = 0; //this should be more descriptive of its function
short goingToPoint = 0;

float radToDegrees = 180/3.14159;
float scalar = 5280;
float o; //find expected wind direction if wind direction can't be calculated in air.

int pointsPerQuarter = 7;
void setup() {
  // put your setup code here, to run once:

}

void loop() {


 

}

void nav(){


  
}


double dist(float lat1, float lon1, float lat2, float lon2){ //may want to use latToX and lonToY functions below
  
  float R = 20925524.9;

  float Lat1 = lat1/radToDegrees;
  float Lon1 = lon1/radToDegrees;
  float Lat2 = lat2/radToDegrees;
  float Lon2 = lon2/radToDegrees;


  float DiffLat = (lat2-lat1)/radToDegrees;
  float DiffLon = (lon2-lon1)/radToDegrees;

  float aX = sin(Lat2/2) * sin(Lat2/2);
  double x = R * 2 * atan2(sqrt(aX),sqrt(1-aX));

  float aY = sin(Lon2/2) * sin(Lon2/2);
  double y = R * 2 * atan2(sqrt(aY),sqrt(1-aY));

  float a = sin(DiffLat/2) * sin(DiffLat/2) + cos(Lat1) * cos(Lat2) * sin(DiffLon/2) * sin(DiffLon/2);
  float c = 2 * atan2(sqrt(a),sqrt(1-a));
  return R * c;
}

float latToX(float gliderLat) //calc distance from lat 0
{
  float aX = sin(gliderLat/2) * sin(gliderLat/2);
  double x = 20925525 * 2 * atan2(sqrt(aX), sqrt(1-aX));
  if(gliderLat<0)
    x=-x;  
  return x;
}

float lonToY(float gliderLon) //calc distance from lon 0
{
  //calc difference in y (long)
  float aY = sin(gliderLon/2) * sin(gliderLon/2);
  double y = 20925525 * 2 * atan2(sqrt(aY), sqrt(1-aY));
  if(gliderLon<0)
    y=-y;
  return y;
}

void back2Path(){
  float offset = 60/radToDegrees; 
  int is = -1;
  float dist = 1000000;
  
  for(int i = 0; i < 56; i+=2){

  float ang = atan2(points[i]*scalar-pos[0],points[i+1]*scalar-pos[1]);
      
      if(ang < 0)
        ang += 360;
      if(ang >= 360)
        ang -= 360;

         if(offset > fang/radToDegrees - ang){

            float angleChange = fang - ang;
            if(angleChange <-180)
              angleChange += 360;
            if(angleChange>180)
              angleChange -= 360;

           angleChange = abs(angleChange);

          float baseDist = (float)((angleChange*9)*(pow(pos[0]-points[i]*scalar,2)+(pow(pos[1]-points[i+1]*scalar,2))));

          int n = 1;
            for(short j = goingToPoint +1; j<=goingToPoint+9; j++)
              if((j%(pointsPerQuarter*4)==i))
                n=j-goingToPoint+1;


                if(baseDist/((goingToPoint+n)%(pointsPerQuarter*4) == i ? n:1) < dist){
                  goingToPoint=i;
                  dist=baseDist/((goingToPoint+n)%(pointsPerQuarter*4)==i ? n:1);
                  
                }
         }
  }

  target[0] = points[goingToPoint*2]*scalar;
  target[1] = points[(goingToPoint * 2)+1]*scalar;
}

void createPathfindingPoints()
{  
  int quarterOfPoints=7;
  //create point objects
  //points=new float[quarterOfPoints*8];
  //quarter 0 and 2 move in the same direction, however, 1 and 3 move in the opposite direction
  for(int i=0;i<quarterOfPoints;i++) {
    float c=sqrt(a*a-b*b);
    float x=1;
    float y1=tan((90/(quarterOfPoints-i)-90/quarterOfPoints)/radToDegrees);
    float x_1=(float)((a*b)/sqrt(pow(a,2)*pow(y1,2)+pow(b,2)*pow(x,2))*x); //
    float y_1=(float)((a*b)/sqrt(pow(a,2)*pow(y1,2)+pow(b,2)*pow(x,2))*y1);
    //set 1st and 2nd quarters.
    points[i*2] = (x_1+c)*cos(o)-y_1*sin(o);
    points[i*2+1] = y_1*cos(o)+(x_1+c)*sin(o);
    points[quarterOfPoints*4+i*2] =(-x_1+c)*cos(o)+y_1*sin(o);
    points[quarterOfPoints*4+i*2+1] =-y_1*cos(o)+(-x_1+c)*sin(o);
    float x_2;
    float y_2;
    //shift values over by one
    if(i!=0) {
      float y2=tan((90/(i+0)-90/quarterOfPoints)/radToDegrees);
      x_2=-(float)((a*b)/sqrt(pow(a,2)*pow(y2,2)+pow(b,2)*pow(x,2))*x); //
      y_2=(float)((a*b)/sqrt(pow(a,2)*pow(y2,2)+pow(b,2)*pow(x,2))*y2);
    } else {
      x_2=0; //?
      y_2=b;
    }
    //set 1st and 3rd quarters
    points[quarterOfPoints*2+i*2] = (x_2+c)*cos(o)-y_2*sin(o);
    points[quarterOfPoints*2+i*2+1] = y_2*cos(o)+(x_2+c)*sin(o);
    points[quarterOfPoints*6+i*2] = (-x_2+c)*cos(o)+y_2*sin(o);
    points[quarterOfPoints*6+i*2+1] = -y_2*cos(o)+(-x_2+c)*sin(o);
  }
  if(goingToPoint*2>=56)
    goingToPoint=0;
  target[0]=points[goingToPoint*2]*scalar;
  target[1]=points[goingToPoint*2+1]*scalar;
}






