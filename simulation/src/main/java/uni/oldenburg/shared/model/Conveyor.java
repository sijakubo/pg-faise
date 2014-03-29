package uni.oldenburg.shared.model;

import java.io.Serializable;

import com.google.gwt.canvas.client.Canvas;
import com.google.gwt.dom.client.CanvasElement;

/**
 * Conveyor class for positioning and drawhandling
 * 
 * @author Matthias
 */

@SuppressWarnings("serial")
public abstract class Conveyor implements Serializable {
	private final static int rasterSize = 20;
	public static final String TABLE_NAME = "szenario_conveyor";

	private int ID;

	private int x;
	private int y;
	private int width;
	private int height;

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
	 * Align positioning based on rastersize
	 * 
	 * @author Matthias
	 */
	public void setPosition(int x, int y) {
		this.x = x - (x % rasterSize);
		this.y = y - (y % rasterSize);
	}

	public int getX() {
		return this.x;
	}

	public int getY() {
		return this.y;
	}

	public int getWidth() {
		return this.width;
	}

	public int getHeight() {
		return this.height;
	}
	
	protected void setSize(int width, int height) {
		if (this.width == width && this.height == height)
			return;
		
		this.width = width;
		this.height = height;
		
		this.canvas = null;
	}

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
	
	public static int getRastersize() {
		return rasterSize;
	}
}
