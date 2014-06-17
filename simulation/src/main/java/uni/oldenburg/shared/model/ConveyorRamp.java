package uni.oldenburg.shared.model;

import uni.oldenburg.client.view.MainFrameView;
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
	public static final String CONVEYOR_TYPE = "Rampe";
	
	public final static int RAMP_STOREAGE = 0;	
	public final static int RAMP_ENTRANCE = 1;
	public final static int RAMP_EXIT = 2;
	
	private boolean bVertical = true;
	private int numBlocks = 4;
	private int rampType = RAMP_STOREAGE;
	
	private int jobCounter = 0;
	
	public ConveyorRamp() {
		this.packageCountMax = 2;		
		this.setVertical(bVertical);
	}
	
	public ConveyorRamp(int x, int y) {
		this.packageCountMax = 2;		
		setPosition(x, y);
		this.setVertical(bVertical);
	}
	
	/**
	 * get (raster-)position of the entry side 
	 * 
	 * @author Matthias
	 */
	public Point getEntryPosition() {
		int x = this.getX();
		int y = this.getY();
		
		switch(this.getDirection()) {
		case DIRECTION_UP:
			y += this.getHeight();
			break;
		case DIRECTION_LEFT:
			x += this.getWidth();
			break;			
		case DIRECTION_DOWN:
			y -= Conveyor.RASTER_SIZE;
			break;			
		case DIRECTION_RIGHT:
			x -= Conveyor.RASTER_SIZE;
			break;			
		}
		
		return new Point(x, y);
	}
	
	/**
	 * get (raster-)position of the exit side 
	 * 
	 * @author Matthias
	 */
	public Point getExitPosition() {
		int x = this.getX();
		int y = this.getY();
		
		switch(this.getDirection()) {
		case DIRECTION_UP:
			y -= Conveyor.RASTER_SIZE;
			break;
		case DIRECTION_LEFT:
			x -= Conveyor.RASTER_SIZE;			
			break;			
		case DIRECTION_DOWN:
			y += this.getHeight();
			break;			
		case DIRECTION_RIGHT:
			x += this.getWidth();
			break;			
		}
		
		return new Point(x, y);
	}

	/**
	 * draw conveyor canvas incl. entry points 
	 * 
	 * @author Matthias, Christopher
	 */
	@Override	
	protected Canvas createCanvas(Canvas canvas) {
		Context2d context = canvas.getContext2d();
		
		context.setFillStyle(CssColor.make(127, 127, 127));
		context.fillRect(0, 0, getWidth(), getHeight());
		context.fill();
		
		int index = 0;
		
		switch(this.getDirection()) {
			case DIRECTION_UP:
				drawEntry(context, CONVEYOR_COLOR_INPUT, 0, this.getHeight() - ENTRY_BORDER_SIZE, this.getWidth(), ENTRY_BORDER_SIZE);
				drawEntry(context, CONVEYOR_COLOR_OUTPUT, 0, 0, this.getWidth(), ENTRY_BORDER_SIZE);
				
				//packages:
				for (String sid : lstPackage) {
					double x = 0.1 * getWidth();
					double y = 0.025 * getHeight() + index++ * getHeight() / 4;
					
					context.setFillStyle(CssColor.make(0, 63, 127));					
					context.fillRect(x, y, 0.8 * getWidth(), getHeight() / 5);
					context.setFillStyle(CssColor.make(255, 255, 255));
					context.fillText(sid, x + 0.5, y + 12.0);
				}
				break;
			case DIRECTION_LEFT:
				drawEntry(context, CONVEYOR_COLOR_INPUT, this.getWidth() - ENTRY_BORDER_SIZE, 0, ENTRY_BORDER_SIZE, this.getWidth());
				drawEntry(context, CONVEYOR_COLOR_OUTPUT, 0, 0, ENTRY_BORDER_SIZE, this.getWidth());
				
				//packages:
				for (String sid : lstPackage) {
					double x = 0.025 * getWidth() + index++ * getWidth() / 4;
					double y = 0.2 * getHeight();
					
					context.setFillStyle(CssColor.make(0, 63, 127));					
					context.fillRect(x, y, getWidth() / 5, 0.6 * getHeight());
					context.setFillStyle(CssColor.make(255, 255, 255));
					context.fillText(sid, x + 0.5, y + 10.0);
				}
				
				break;
			case DIRECTION_DOWN:
				drawEntry(context, CONVEYOR_COLOR_INPUT, 0, 0, this.getWidth(), ENTRY_BORDER_SIZE);
				drawEntry(context, CONVEYOR_COLOR_OUTPUT, 0, this.getHeight() - ENTRY_BORDER_SIZE, this.getWidth(), ENTRY_BORDER_SIZE);

				index = this.getPackageCountMax();
				
				//packages:
				for (String sid : lstPackage) {
					double x = 0.1 * getWidth();
					double y = 0.025 * getHeight() + index-- * getHeight() / 4;
					
					context.setFillStyle(CssColor.make(0, 63, 127));					
					context.fillRect(x, y, 0.8 * getWidth(), getHeight() / 5);
					context.setFillStyle(CssColor.make(255, 255, 255));
					context.fillText(sid, x + 0.5, y + 12.0);
				}
				
				break;
			case DIRECTION_RIGHT:
				drawEntry(context, CONVEYOR_COLOR_INPUT, 0, 0, ENTRY_BORDER_SIZE, this.getWidth());				
				drawEntry(context, CONVEYOR_COLOR_OUTPUT , this.getWidth() - ENTRY_BORDER_SIZE, 0, ENTRY_BORDER_SIZE, this.getWidth());

				index = this.getPackageCountMax();
				
				//packages:
				for (String sid : lstPackage) {
					double x = 0.025 * getWidth() + index-- * getWidth() / 4;
					double y = 0.2 * getHeight();
					
					context.setFillStyle(CssColor.make(0, 63, 127));					
					context.fillRect(x, y, getWidth() / 5, 0.6 * getHeight());
					context.setFillStyle(CssColor.make(255, 255, 255));
					context.fillText(sid, x + 0.5, y + 10.0);
				}
				
				break;
		}			
		
		return canvas;
	}

	public int getJobCounter() {
		return jobCounter;
	}


	public void setJobCounter(int jobCounter) {
		this.jobCounter = jobCounter;
	}

	@Override
	public String getType() {
		return ConveyorRamp.CONVEYOR_TYPE;
	}
	
	public int getRampType() {
		return rampType;
	}
	
	public int getNumberOfBlocks() {
		return numBlocks;
	}
		
	/**
	 * auto-set type based on location of conveyor
	 * 
	 * @author Matthias
	 */
	public void setPosition(int x, int y) {
		super.setPosition(x, y);
		
		assignRampType();	
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
		
		assignRampType();
		
		return bResult;
	}
	
	/**
	 * set ramp type based in location and direction
	 * 
	 * @author Matthias
	 */
	private void assignRampType() {
		rampType = RAMP_STOREAGE;
		
		switch(getDirection()) {
			case DIRECTION_UP:
				if (getY() == 0)
					rampType = RAMP_EXIT;	
				else if (getY() == (MainFrameView.canvasHeight - getHeight()))  
					rampType = RAMP_ENTRANCE;
					
				break;
			case DIRECTION_RIGHT:
				if (getX() == 0)
					rampType = RAMP_ENTRANCE;	
				else if (getX() == (MainFrameView.canvasWidth - getWidth())) 
					rampType = RAMP_EXIT;
					
				break;
			case DIRECTION_DOWN:
				if (getY() == 0)
					rampType = RAMP_ENTRANCE;	
				else if (getY() == (MainFrameView.canvasHeight - getHeight()))  
					rampType = RAMP_EXIT;
				
				break;
			case DIRECTION_LEFT:
				if (getX() == 0)
					rampType = RAMP_EXIT;
				else if (getX() == (MainFrameView.canvasWidth - getWidth()))
					rampType = RAMP_ENTRANCE;
				
				break;
		}
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
