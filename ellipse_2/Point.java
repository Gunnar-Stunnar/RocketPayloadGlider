public class Point {
  
  private float x;
  private float y;
  boolean regen;
  
  public Point(float _x, float _y, boolean _regen) {
    x=_x;
    y=_y;
    regen=_regen;
  }
  
  public float getX() { return x; }
  public float getY() { return y; }
  public boolean getRegen() { return regen; }
  
  public void set(float _x, float _y) {
    x=_x;
    y=_y;
  }
  public void invertX() { x*=-1; }
  public void invertY() { y*=-1; }
  
  
}