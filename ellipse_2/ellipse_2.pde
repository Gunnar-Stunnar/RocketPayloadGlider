
Point plane;

float a = 2*5280;
float b = 1*5280 + (a/4);
float c = (float)Math.sqrt(a*a-b*b);

float t = 0;
Point[] points;
int goingToPoint=0;
float scalar=.019;
int range = 60;

float radToDegrees = 180/3.14159;

//angle offset
float o;

int pointsPerQuarter = 7;
boolean autoDecrement= true;

Flyer f;
int secondsSinceFUpdate;

boolean trace = true;
boolean showDestination = false;

void setup(){
 size(1000,1000);
 background(0);
 //create all points
  plane = new Point(random(-(width/2),width/2),random(-(height/2),height/2),true);
 f=new Flyer(plane.getX(),plane.getY(),0,0,0);
 
 secondsSinceFUpdate=millis()/10;
 
// frameRate(2);
   
}

void draw(){
  if(trace)
    background(0);
  stroke(255);
  line(width/2,0,width/2,height);
  line(0,height/2,width,height/2);
  
 
 translate(width/2,height/2);
 
 //-----------------------------------------------------------------test code for glider
 
 
 o = (atan2(plane.getY(),plane.getX()))+3.14159/2;
 createPoints();
 
 
 
 println(millis());
 stroke(255,0,0);
 //line(-1500,0,1500,0);
 //line(0,-1500,0,1500);
 stroke(255,0,0);
 
 for(Point p : points) {
   point(p.getX()*scalar,p.getY()*scalar);
   //delay(1000);
 }
 
 point(plane.getX(),plane.getY());
 point(0,0);
 
 

 
 println(f.getAngle());
 
 stroke(255,0,255);
 //line(mouseX-width/2,mouseY-height/2,points[goingToPoint].getX()*scalar,points[goingToPoint].getY()*scalar);
 
 if(Math.sqrt(Math.pow(f.getDX()-f.getX(),2)+Math.pow(f.getDY()-f.getY(),2)) > range*2 || (f.getAngleChange() > 65 && Math.sqrt(Math.pow(f.getDX()-f.getX(),2)+Math.pow(f.getDY()-f.getY(),2)) > range*1.25))
   back2Path();
   
 if(Math.sqrt(Math.pow(points[goingToPoint].getX()*scalar-(f.getX()),2)+Math.pow(points[goingToPoint].getY()*scalar-(f.getY()),2))<range) {
   goingToPoint++;
   if(goingToPoint==points.length)
     goingToPoint=0;
     
     
   if(points[goingToPoint].getRegen() && a!=b && autoDecrement) {
     a-=0.1;
     b = 1*5280 + a/4;
     if(a<b)
       a=b;
     c = (float)Math.sqrt(a*a-b*b);
     createPoints();
     
     
   }
   f.setDestination(points[goingToPoint].getX()*scalar,points[goingToPoint].getY()*scalar);
 }
 
 if(millis()/10 - secondsSinceFUpdate >= 5) {
   System.out.print("Flyer update");
   f.move();
   secondsSinceFUpdate=millis()/10;
 }
 stroke(0,255,(a/2)*255);
 line(f.getX(),f.getY(),f.getX()+(20)*cos(f.getAngle()/radToDegrees)-0*sin(f.getAngle()/radToDegrees),f.getY()+0*cos(f.getAngle()/radToDegrees)+(20)*sin(f.getAngle()/radToDegrees));
 point(f.getX(),f.getY());
 stroke(255,0,255);
 if(showDestination)
   line(f.getX(),f.getY(),f.getDX(),f.getDY());
 
}

//Some stuff to allow scenerio testing
void keyPressed() {
  System.out.print(key);
  if(key=='a'||key=='A') {
    System.out.println('p');
    a-=.1;
    if(a==0)
      a+=.1;
    c = (float)Math.sqrt(a*a-b*b);
    createPoints();
  } else if(key=='d'||key=='D') {
    System.out.println('p');
    a+=.1;
    c = (float)Math.sqrt(a*a-b*b);
    createPoints();
  } else if(key=='s'||key=='S') {
    System.out.println('p');
    b-=.1;
    if(b==0)
      b+=.1;
    c = (float)Math.sqrt(a*a-b*b);
    createPoints();
  } else if(key=='w'||key=='W') {
    System.out.println('p');
    b+=.1;
    c = (float)Math.sqrt(a*a-b*b);
    createPoints();
  } else if(key=='q'||key=='Q') {
    System.out.println('p');
    o+=1/radToDegrees;
    createPoints();
  } else if(key=='e'||key=='E') {
    System.out.println('p');
    o-=1/radToDegrees;
    createPoints();
  } else if(key=='z'||key=='Z') {
    System.out.println('p');
    if(pointsPerQuarter!=1) {
      pointsPerQuarter--;
      createPoints();
    }
  } else if(key=='x'||key=='X') {
    System.out.println('p');
    pointsPerQuarter++;
    createPoints();
  } else if(key=='c'||key=='C') {
    autoDecrement=!autoDecrement;
  } else if(key=='t'||key=='T') {
    trace=!trace;
  } else if(key=='r'||key=='R') {
    showDestination=!showDestination;
  }
}


void createPoints() {
  //Get number of points based on altitude, num must be divisble by 4
  
  int quarterOfPoints=pointsPerQuarter;
  //create point objects
  points = new Point[pointsPerQuarter*4];
  /*for(int i=0;i<numOfPoints;i++) {
    points[i] = new Point((float)0,(float)0,false);
  }*/
  //quarter 0 and 2 move in the same direction, however, 1 and 3 move in the opposite direction
  for(int i=0;i<quarterOfPoints;i++) {
    float x=1;
    float y1=tan((90/(quarterOfPoints-i)-90/quarterOfPoints)/radToDegrees);
    
    float x_1=(float)((a*b)/Math.sqrt(Math.pow(a,2)*Math.pow(y1,2)+Math.pow(b,2)*Math.pow(x,2))*x); //
    float y_1=(float)((a*b)/Math.sqrt(Math.pow(a,2)*Math.pow(y1,2)+Math.pow(b,2)*Math.pow(x,2))*y1);
    //set 1st and 2nd quarters.
    points[i] = new Point((x_1+c)*cos(o)-y_1*sin(o),y_1*cos(o)+(x_1+c)*sin(o),i == 0);
    points[quarterOfPoints*2+i] = new Point((-x_1+c)*cos(o)+y_1*sin(o),-y_1*cos(o)+(-x_1+c)*sin(o),false);
    
    float x_2;
    float y_2;
    //shift values over by one
    if(i!=0) {
      float y2=tan((90/(i+0)-90/quarterOfPoints)/radToDegrees);
      x_2=-(float)((a*b)/Math.sqrt(Math.pow(a,2)*Math.pow(y2,2)+Math.pow(b,2)*Math.pow(x,2))*x); //
      y_2=(float)((a*b)/Math.sqrt(Math.pow(a,2)*Math.pow(y2,2)+Math.pow(b,2)*Math.pow(x,2))*y2);
    } else {
      float y2=1;
      x=0;
      x_2=(float)((a*b)/Math.sqrt(Math.pow(a,2)*Math.pow(1,2)+Math.pow(b,2)*Math.pow(0,2))*0); //?
      y_2=(float)((a*b)/Math.sqrt(Math.pow(a,2)*Math.pow(1,2)+Math.pow(b,2)*Math.pow(0,2))*1);
    }
    
    //set 1st and 3rd quarters
    points[quarterOfPoints+i] = new Point((x_2+c)*cos(o)-y_2*sin(o),y_2*cos(o)+(x_2+c)*sin(o),false);
    points[quarterOfPoints*3+i] = new Point((-x_2+c)*cos(o)+y_2*sin(o),-y_2*cos(o)+(-x_2+c)*sin(o),false);
  }
  if(goingToPoint>=points.length)
     goingToPoint=0;
  f.setDestination(points[goingToPoint].getX()*scalar,points[goingToPoint].getY()*scalar);
}


void back2Path(){
      /*float offset = 10/radToDegrees;  
  
  //line(f.getX(),f.getY(),f.getX() + cos((f.getAngle()/radToDegrees) + (offset))*100,f.getY() + sin((f.getAngle()/radToDegrees)+ (offset))*100);
  //line(f.getX(),f.getY(),f.getX() + cos((f.getAngle()/radToDegrees) - (offset))*100,f.getY() + sin((f.getAngle()/radToDegrees)- (offset))*100);
  
  float dist = 1000000;
  int is = -1;
  float angleIs = 90;
  
      for(int i = 0; i < points.length; i++){
        
      float ang = atan2(points[i].getY()*scalar-f.getY(),points[i].getX()*scalar-f.getX());
      if(ang<0)
        ang+=360;
      if(ang>=360)
        ang-=360;
      
      println("fffffffffff " + ang + "," + (f.getAngle()/radToDegrees-offset) + " " + (f.getAngle()/radToDegrees+offset));
      if( ang <= f.getAngleChange()/radToDegrees + offset && ang >= f.getAngleChange()/radToDegrees - offset){
      //if(Math.sqrt((Math.pow(f.getX()-points[i].getX()*scalar,2)+(Math.pow(f.getY()-points[i].getY()*scalar,2)))) > 400) {
        
        /*if(Math.sqrt((Math.pow(f.getX(),2)-Math.pow(points[i].getX()*scalar,2))+(Math.pow(f.getY(),2)-Math.pow(points[i].getY()*scalar,2))) < dist){
            dist = (float)Math.sqrt((Math.pow(f.getX(),2)-Math.pow(points[i].getX()*scalar,2))+(Math.pow(f.getY(),2)-Math.pow(points[i].getY()*scalar,2)));
            is = i;
        }*//*
        
        if((Math.sqrt((Math.pow(f.getX()-points[i].getX()*scalar,2)+(Math.pow(f.getY()-points[i].getY()*scalar,2)))) < dist)&&(f.getAngleChange()<angleIs)) {
            dist = (float)Math.sqrt((Math.pow(f.getX(),2)-Math.pow(points[i].getX()*scalar,2))+(Math.pow(f.getY(),2)-Math.pow(points[i].getY()*scalar,2)));
            is = i;
        }
        
          ellipse(points[i].getX()*scalar,points[i].getY()*scalar,5,5);
          
          
      }
      
          
        }  
   
      
      if(is != -1) {
            f.setDestination(points[is].getX()*scalar,points[is].getY()*scalar);
            goingToPoint=is;
            
            if(goingToPoint>=points.length)
              goingToPoint=0;
            else if(goingToPoint<0)
              goingToPoint=points.length-1;
      }*/
      float angleDivider = .111111;
      float offset = 60/radToDegrees;
      int is = -1;
      float dist = 10000000;
      for(int i = 0; i < points.length; i++){
        
        
      float ang = atan2(points[i].getY()*scalar-f.getY(),points[i].getX()*scalar-f.getX());
      if(ang<0)
        ang+=360;
      if(ang>=360)
        ang-=360;
      
     if( offset > f.getAngle()/radToDegrees - ang){
      
      
      float angleChange= f.getAngle()-ang;
        if(angleChange<-180)
          angleChange+=360;
        if(angleChange>180)
          angleChange-=360;
      angleChange=abs(angleChange);
      
      float baseDist = (float)((angleChange/angleDivider)*(Math.pow(f.getX()-points[i].getX()*scalar,2)+(Math.pow(f.getY()-points[i].getY()*scalar,2))));
      
      /*if((angleChange/angleDivider)*(Math.pow(f.getX()-points[i].getX()*scalar,2)+(Math.pow(f.getY()-points[i].getY()*scalar,2)))/((goingToPoint+(i%pointsPerQuarter))%(pointsPerQuarter*4)==i ? (goingToPoint+pointsPerQuarter-(i%pointsPerQuarter)+2):1) < dist) {
        ellipse(points[i].getX()*scalar,points[i].getY()*scalar,5,5);
        goingToPoint=i;
        System.out.println("NNNNNNNNNNNN");
        dist=(float)((angleChange/angleDivider)*(Math.pow(f.getX()-points[i].getX()*scalar,2)+(Math.pow(f.getY()-points[i].getY()*scalar,2))))/((goingToPoint+(i%pointsPerQuarter))%(pointsPerQuarter*4)==i ? (goingToPoint+pointsPerQuarter-(i%pointsPerQuarter)+2):1);
      }*/
      
      //if value equals goingToPoint + x % totalPoints, set n to x, otherwise n = 1
      int n = 1;
      for(int j = goingToPoint+1; j<=goingToPoint+9; j++) {
        if((j%(pointsPerQuarter*4))==i)
          n=j-goingToPoint+1;
      }
      
      
      if(baseDist/((goingToPoint+n)%(pointsPerQuarter*4)==i ? n:1) < dist) {
        ellipse(points[i].getX()*scalar,points[i].getY()*scalar,5,5);
        goingToPoint=i;
        System.out.println("NNNNNNNNNNNN");
        dist=baseDist/((goingToPoint+n)%(pointsPerQuarter*4)==i ? n:1);
      }
      }
      
      }
      
      f.setDestination(points[goingToPoint].getX()*scalar,points[goingToPoint].getY()*scalar);
      
    }
  
  
  
  