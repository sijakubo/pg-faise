package uni.oldenburg.shared.model;

import java.io.Serializable;

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
	private final static int rasterSize = 20;
	
	public static final int DIRECTION_UP 	= 0;
	public static final int DIRECTION_LEFT 	= 1;
	public static final int DIRECTION_DOWN 	= 2;
	public static final int DIRECTION_RIGHT = 3;
	
	protected static final int ENTRY_BORDER_SIZE = rasterSize / 4;	
	protected static final String CONVEYOR_COLOR_ENTER = "red";
	protected static final String CONVEYOR_COLOR_EXIT  = "green";	

	private int ID;

	private int x;
	private int y;
	private int width;
	private int height;
	
	private int direction = DIRECTION_UP;

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

	/**
	 * get position based on raster layout
	 * 
	 * @author Matthias
	 */
	public int getX() {
		return this.x  - (this.x % rasterSize);
	}

	public int getY() {
		return this.y - (this.y % rasterSize);
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
		return rasterSize;
	}
}
