package uni.oldenburg.shared.model;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

import uni.oldenburg.server.agent.helper.DelayTimes;
import uni.oldenburg.shared.model.event.EventHelper;
import uni.oldenburg.shared.model.event.PositionChangedEvent;

import com.google.gwt.canvas.client.Canvas;
import com.google.gwt.canvas.dom.client.Context2d;
import com.google.gwt.canvas.dom.client.CssColor;
import com.google.gwt.dom.client.CanvasElement;

/**
 * Conveyor class for positioning and drawhandling
 * 
 * @author Matthias
 */

@SuppressWarnings("serial")
public abstract class Conveyor implements Serializable {
	public static final String TABLE_NAME = "szenario_conveyor";	
	public final static int RASTER_SIZE = 20;
	
	public static final int DIRECTION_UP 	= 0;
	public static final int DIRECTION_RIGHT = 1;	
	public static final int DIRECTION_DOWN 	= 2;
	public static final int DIRECTION_LEFT 	= 3;	
	
	protected static final int ENTRY_BORDER_SIZE = RASTER_SIZE / 8;	
	protected static final String CONVEYOR_COLOR_INPUT = "red";
	protected static final String CONVEYOR_COLOR_OUTPUT  = "green";	
	
	protected boolean hasIncomingJob = false;
	protected boolean hasOutgoingJob = false;

	private int ID;

	private int x;
	private int y;
	private int width;
	private int height;
	
	private int direction = DIRECTION_UP;

	protected List<String> lstPackage = new ArrayList<String>(); 
	
	protected int packageCountMax = 0;

	private transient Canvas canvas;
	protected String strType;

	private static int ID_Counter = 0;

	public abstract String getType();
	protected abstract Canvas createCanvas(Canvas canvas);

    protected Conveyor() {
		ID = ++ID_Counter;
		strType = getType();
		
		setPosition(0, 0);
		setSize(0, 0);
    }

    protected Conveyor(int x, int y, int width, int height) {
		ID = ++ID_Counter;
		strType = getType();

		setPosition(x, y);
		setSize(width, height);
	}
    
    public static void resetCounter() {
    	Conveyor.ID_Counter = 0;
    }
    
	public boolean hasIncomingJob() {
		return hasIncomingJob;
	}
	public void setIncomingJob(boolean hasIncomingJob) {
		this.hasIncomingJob = hasIncomingJob;
	}
	public boolean hasOutgoingJob() {
		return hasOutgoingJob;
	}
	public void setOutgoingJob(boolean hasOutgoingJob) {
		this.hasOutgoingJob = hasOutgoingJob;
	}

	public int getID() {
		return ID;
	}
	
	/**
	 * rotate conveyor clockwise 
	 * 
	 * @author Matthias
	 */
	public void rotateClockwise() {		
		this.rotate(++this.direction % 4);
	}
	
	/**
	 * rotate conveyor based on given direction 
	 * 
	 * @author Matthias
	 */
	public boolean rotate(int direction) {
		if (direction > 3 || direction < 0)
			return false;
		
		this.direction = direction;
		this.canvas = null;
		
		return true;
	}
	
	public int getDirection() {
		return direction;
	}

	/**
	 * set positioning
	 * 
	 * @author Matthias
	 */
	public void setPosition(int x, int y) {
		this.x = x;
		this.y = y;
	}
	
	public void setPosition(int x, int y, boolean onServer) {
		this.setPosition(x, y);
		
		if (onServer) {
			EventHelper.WaitForMS(DelayTimes.DRIVE_TO_DELAY);
			EventHelper.addEvent(new PositionChangedEvent(x, y, this.getID()));
		}
	}

	/**
	 * get position based on raster layout
	 * 
	 * @author Matthias
	 */
	public int getX() {
		return this.x  - (this.x % RASTER_SIZE);
	}

	public int getY() {
		return this.y - (this.y % RASTER_SIZE);
	}
	
	//------------------------------------

	public int getWidth() {
		return this.width;
	}

	public int getHeight() {
		return this.height;
	}
	
	/**
	 * reinit canvas drawing on resize
	 * 
	 * @author Matthias
	 */
	protected void setSize(int width, int height) {
		if (this.width == width && this.height == height)
			return;
		
		this.width = width;
		this.height = height;
		
		this.canvas = null;
	}

	/**
	 * move via relative position values
	 * 
	 * @author Matthias
	 */
	public void move(int x_rel, int y_rel) {
		this.x += x_rel;
		this.y += y_rel;
	}

	/**
	 * draw and retrieve canvas when necessary
	 * 
	 * @author Matthias
	 */
	public CanvasElement getCanvasElement() {
		if (canvas == null) {
			canvas = Canvas.createIfSupported();

			if (canvas != null) {
				canvas.setWidth(width + "px");
				canvas.setHeight(height + "px");
				canvas.setCoordinateSpaceWidth(width);
				canvas.setCoordinateSpaceHeight(height);

				canvas = createCanvas(canvas);			
			}
		}

		return canvas.getCanvasElement();
	}
	
	/**
	 * helper function to draw entry points of conveyors 
	 * 
	 * @author Matthias
	 */
	protected void drawEntry(Context2d context, String color, int x, int y, int width, int height) {
		context.setFillStyle(CssColor.make(color));
		context.fillRect(x, y, width, height);
		context.fill();
	}	
	
	public static int getRastersize() {
		return RASTER_SIZE;
	}
	
	public int getPackageCount() {
		return lstPackage.size();
	}
	
	public void addPackage(String sid) {
		lstPackage.add(sid);
		
		this.canvas = null;
	}
	
	public void removePackage(String sid) {
		lstPackage.remove(sid);
		
		this.canvas = null;
	}

	public int getPackageCountMax() {
		return packageCountMax;
	}
}
