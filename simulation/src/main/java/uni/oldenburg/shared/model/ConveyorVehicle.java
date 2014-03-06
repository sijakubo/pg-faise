package uni.oldenburg.shared.model;

import com.google.gwt.canvas.client.Canvas;
import com.google.gwt.canvas.dom.client.Context2d;
import com.google.gwt.canvas.dom.client.CssColor;

@SuppressWarnings("serial")
public class ConveyorVehicle extends Conveyor {
	public static final String TYPE = "Fahrzeug";
	
	public ConveyorVehicle() {
		super(0, 0, 20, 20);
	}
	
	public ConveyorVehicle(int x, int y) {
		super(x, y, 20, 20);
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
