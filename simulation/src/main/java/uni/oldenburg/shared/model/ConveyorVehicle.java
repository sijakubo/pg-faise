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

	@Override
	protected Canvas createCanvas(Canvas canvas) {
		Context2d context = canvas.getContext2d();
		
		context.setFillStyle(CssColor.make("blue"));
		context.fillRect(0, 0, getWidth(), getHeight());
		context.fill();
		
		return canvas;
	}

	@Override
	public String getType() {
		return ConveyorVehicle.TYPE;
	}

}
