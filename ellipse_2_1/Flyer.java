public class Flyer {
  private float xPos, yPos, xDestination, yDestination, angle;
  
  private float maxAngleChange=(float).5;
  
  private double windDirection=Math.random()*3.14159*2, windSpeed=0.019*(Math.random()*21+1)/250, windDX=windSpeed*Math.cos(windDirection),currentWindDX=0,windDY=windSpeed*Math.sin(windDirection),currentWindDY=0,timeTillWindChange=5,timeLastChange=0;
  
  private float speed = (float)(11*.019)/250;
   
  public Flyer(float _xPos, float _yPos, float _xDestination, float _yDestination, float _angle) {
    xPos=_xPos;
    yPos=_yPos;
    xDestination=_xDestination;
    yDestination=_yDestination;
    angle=_angle;
  }
  
  private void changeAngle() {
    float angleToTarget=(float)(Math.atan2((double)(yDestination-yPos),(double)(xDestination-xPos))*(180/3.14159));
    
    if(angleToTarget<0)
      angleToTarget+=360;
    if(angleToTarget>=360)
      angleToTarget-=360;
    
    float angleChange= angle-angleToTarget;
    if(angleChange<-180)
      angleChange+=360;
    if(angleChange>180)
      angleChange-=360;
    
    if(angleChange>maxAngleChange) {
      angle-=maxAngleChange;
    } else if(angleChange<-maxAngleChange) {
      angle+=maxAngleChange;
    } else {
      angle-=angle-angleToTarget;
    }
    
    if(angle<0)
      angle+=360;
    if(angle>=360)
      angle-=360;
    System.out.println(" "+angleToTarget+" "+angle+" "+(angle-angleToTarget));
  }
  
  public void move() {
    //xPos+(20)*Math.cos(angle/(180/3.14159))-0*Math.sin(angle/(180/3.14159)),yPos+0*Math.cos(angle/(180/3.14159))+(20)*Math.sin(angle/(180/3.14159))
    xPos+=Math.cos(angle/(180/3.14159))*speed+currentWindDX;
    yPos+=Math.sin(angle/(180/3.14159))*speed+currentWindDY;
    changeAngle();
    changeWind();
  }
  
  private void changeWind() {
    if(timeLastChange%timeTillWindChange == 0) {
      if(currentWindDX==windDX) {
        currentWindDX=0;
      } else {
        currentWindDX=windDX;
      }
      if(currentWindDY==windDY) {
        currentWindDY=0;
      } else {
        currentWindDY=windDY;
      }
      timeTillWindChange=(int)(Math.random()*50)+1;
    }
    timeLastChange++;
  }
  
  
  public float getAngle() { return angle; }
  public float getX() { return xPos; }
  public float getY() { return yPos; }
  public float getDX() { return xDestination; }
  public float getDY() { return yDestination; }
  
  public float getAngleChange() {
    float angleToTarget=(float)(Math.atan2((double)(yDestination-yPos),(double)(xDestination-xPos))*(180/3.14159));
    
    if(angleToTarget<0)
      angleToTarget+=360;
    if(angleToTarget>=360)
      angleToTarget-=360;
    
    float angleChange= angle-angleToTarget;
    if(angleChange<-180)
      angleChange+=360;
    if(angleChange>180)
      angleChange-=360;
    return angleChange;
  }
  
  public void setDestination(float _xDestination, float _yDestination) {
    xDestination=_xDestination;
    yDestination=_yDestination;
  }
}