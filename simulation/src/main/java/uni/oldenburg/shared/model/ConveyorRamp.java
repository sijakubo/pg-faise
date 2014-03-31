package uni.oldenburg.shared.model;

import com.google.gwt.canvas.client.Canvas;
import com.google.gwt.canvas.dom.client.Context2d;
import com.google.gwt.canvas.dom.client.CssColor;

/**
 * class for handing ramp blocks in the canvas
 * 
 * @author Matthias
 */

@SuppressWarnings("serial")
public class ConveyorRamp extends Conveyor {
	public static final String TYPE = "Rampe";
	private boolean bVertical = true;
	private int numBlocks = 3;

	public ConveyorRamp() {
		this.setVertical(bVertical);
	}
	
	public ConveyorRamp(int x, int y) {
		setPosition(x, y);
		this.setVertical(bVertical);
	}

	/**
	 * draw conveyor canvas incl. entry points 
	 * 
	 * @author Matthias
	 */
	@Override	
	protected Canvas createCanvas(Canvas canvas) {
		Context2d context = canvas.getContext2d();
		
		context.setFillStyle(CssColor.make("purple"));
		context.fillRect(0, 0, getWidth(), getHeight());
		context.fill();
		
		switch(this.getDirection()) {
			case DIRECTION_UP:
				drawEntry(context, CONVEYOR_COLOR_ENTER, 0, 0, this.getWidth(), ENTRY_BORDER_SIZE);
				drawEntry(context, CONVEYOR_COLOR_EXIT , 0, this.getHeight() - ENTRY_BORDER_SIZE, this.getWidth(), ENTRY_BORDER_SIZE);
				break;
			case DIRECTION_LEFT:
				drawEntry(context, CONVEYOR_COLOR_ENTER, this.getWidth() - ENTRY_BORDER_SIZE, 0, ENTRY_BORDER_SIZE, this.getWidth());
				drawEntry(context, CONVEYOR_COLOR_EXIT , 0, 0, ENTRY_BORDER_SIZE, this.getWidth());
				break;
			case DIRECTION_DOWN:
				drawEntry(context, CONVEYOR_COLOR_ENTER, 0, this.getHeight() - ENTRY_BORDER_SIZE, this.getWidth(), ENTRY_BORDER_SIZE);				
				drawEntry(context, CONVEYOR_COLOR_EXIT , 0, 0, this.getWidth(), ENTRY_BORDER_SIZE);
				break;
			case DIRECTION_RIGHT:
				drawEntry(context, CONVEYOR_COLOR_ENTER, 0, 0, ENTRY_BORDER_SIZE, this.getWidth());				
				drawEntry(context, CONVEYOR_COLOR_EXIT , this.getWidth() - ENTRY_BORDER_SIZE, 0, ENTRY_BORDER_SIZE, this.getWidth());
				break;
		}			
		
		return canvas;
	}

	@Override
	public String getType() {
		return ConveyorRamp.TYPE;
	}
	
	/**
	 * flip vertical status based in entry rotation angle 
	 * 
	 * @author Matthias
	 */
	public void rotateClockwise() {
		super.rotateClockwise();
		
		this.setVertical(((this.getDirection() % 2) == 0));
	}
	
	/**
	 * flip vertical status based on given entry direction 
	 * 
	 * @author Matthias
	 */
	public boolean rotate(int direction) {
		boolean bResult = super.rotate(direction);
		
		if (bResult)
			this.setVertical(((direction % 2) == 0));
		
		return bResult;
	}
	
	/**
	 * set vertival status 
	 * 
	 * @author Matthias
	 */
	private void setVertical(boolean value) {
		this.bVertical = value;
		
		if (this.bVertical)
			this.setSize(Conveyor.getRastersize(), Conveyor.getRastersize() * numBlocks);
		else
			this.setSize(Conveyor.getRastersize() * numBlocks, Conveyor.getRastersize());
	}

	public boolean isVertical() {
		return bVertical;
	}
}
