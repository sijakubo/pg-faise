package uni.oldenburg.shared.model;

import com.google.gwt.canvas.client.Canvas;
import com.google.gwt.canvas.dom.client.Context2d;
import com.google.gwt.canvas.dom.client.CssColor;

/**
 * class for handing vehicle blocks in the canvas
 * 
 * @author Matthias
 */

@SuppressWarnings("serial")
public class ConveyorVehicle extends Conveyor {
	public static final String CONVEYOR_TYPE = "Fahrzeug";
	private double batteryCharge = Math.random();
	
	public ConveyorVehicle() {
		super(0, 0, Conveyor.getRastersize(), Conveyor.getRastersize());
		this.packageCountMax = 1;
	}
	
	public ConveyorVehicle(int x, int y) {
		super(x, y, Conveyor.getRastersize(), Conveyor.getRastersize());
		this.packageCountMax = 1;		
	}

	/**
	 * helper function to draw entry points of conveyors 
	 * 
	 * @author Matthias, Christopher
	 */
	@Override
	protected Canvas createCanvas(Canvas canvas) {
		Context2d context = canvas.getContext2d();
		
		context.setFillStyle(CssColor.make(127, 127, 127));
		context.fillRect(0, 0, getWidth(), getHeight());
		context.fill();
		
		switch(this.getDirection()) {
			case DIRECTION_UP:
				context.setFillStyle(CssColor.make(0, 50, 75));
				context.fillRect(getWidth() / 4, 0, getWidth() / 2, getHeight());
				drawEntry(context, CONVEYOR_COLOR_INPUT, 0, 0, this.getWidth(), ENTRY_BORDER_SIZE);
				drawEntry(context, CONVEYOR_COLOR_OUTPUT , 0, this.getHeight() - ENTRY_BORDER_SIZE, this.getWidth(), ENTRY_BORDER_SIZE);
				break;
			case DIRECTION_LEFT:
				context.setFillStyle(CssColor.make(0, 50, 75));
				context.fillRect(0, getHeight() / 4, getWidth(), getHeight() / 2);
				drawEntry(context, CONVEYOR_COLOR_INPUT, this.getWidth() - ENTRY_BORDER_SIZE, 0, ENTRY_BORDER_SIZE, this.getWidth());
				drawEntry(context, CONVEYOR_COLOR_OUTPUT , 0, 0, ENTRY_BORDER_SIZE, this.getWidth());
				break;
			case DIRECTION_DOWN:
				context.setFillStyle(CssColor.make(0, 50, 75));
				context.fillRect(getWidth() / 4, 0, getWidth() / 2, getHeight());
				drawEntry(context, CONVEYOR_COLOR_INPUT, 0, this.getHeight() - ENTRY_BORDER_SIZE, this.getWidth(), ENTRY_BORDER_SIZE);				
				drawEntry(context, CONVEYOR_COLOR_OUTPUT , 0, 0, this.getWidth(), ENTRY_BORDER_SIZE);
				break;
			case DIRECTION_RIGHT:
				context.setFillStyle(CssColor.make(0, 50, 75));
				context.fillRect(0, getHeight() / 4, getWidth(), getHeight() / 2);
				drawEntry(context, CONVEYOR_COLOR_INPUT, 0, 0, ENTRY_BORDER_SIZE, this.getWidth());				
				drawEntry(context, CONVEYOR_COLOR_OUTPUT , this.getWidth() - ENTRY_BORDER_SIZE, 0, ENTRY_BORDER_SIZE, this.getWidth());
				break;
		}		
		
		//package:
		for (String sid : lstPackage) {
			double x = 0.1 * getWidth();
			double y = 0.1 * getHeight();
			
			context.setFillStyle(CssColor.make(0, 63, 127));					
			context.fillRect(x, y, 0.8 * getWidth(), 0.8 * getHeight());
			context.setFillStyle(CssColor.make(255, 255, 255));
			context.fillText(sid, x + 0.5, y + 12.0);
		}
		
		return canvas;
	}
	
	public double getBatteryCharge() {
		return batteryCharge;
	}

	@Override
	public String getType() {
		return ConveyorVehicle.CONVEYOR_TYPE;
	}

}
