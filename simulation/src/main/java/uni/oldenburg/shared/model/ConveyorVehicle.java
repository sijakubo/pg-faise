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
	public static final String TYPE = "Fahrzeug";
	
	public ConveyorVehicle() {
		super(0, 0, Conveyor.getRastersize(), Conveyor.getRastersize());
	}
	
	public ConveyorVehicle(int x, int y) {
		super(x, y, Conveyor.getRastersize(), Conveyor.getRastersize());
	}

	/**
	 * helper function to draw entry points of conveyors 
	 * 
	 * @author Matthias
	 */
	@Override
	protected Canvas createCanvas(Canvas canvas) {
		Context2d context = canvas.getContext2d();
		
		context.setFillStyle(CssColor.make("blue"));
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
		return ConveyorVehicle.TYPE;
	}

}