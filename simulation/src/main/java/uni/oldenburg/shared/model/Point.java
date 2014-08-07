package uni.oldenburg.shared.model;

import jade.util.leap.Serializable;
@SuppressWarnings("serial")
public class Point implements Serializable {
	private int x = 0;
	private int y = 0;
	
	public Point() {}
	
	public Point(int x, int y) {
		this.x = x;
		this.y = y;
	}
	
	public int getX() {
		return x;
	}
	public void setX(int x) {
		this.x = x;
	}
	public int getY() {
		return y;
	}
	public void setY(int y) {
		this.y = y;
	}
	
	public String toString() {
		return "X/Y: " + x + ":" + y;
	}
}
