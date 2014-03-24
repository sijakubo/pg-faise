package uni.oldenburg.shared.model;

import com.google.gwt.canvas.client.Canvas;
import com.google.gwt.canvas.dom.client.Context2d;
import com.google.gwt.canvas.dom.client.CssColor;

@SuppressWarnings("serial")
public class ConveyorRamp extends Conveyor {
	public static final String TYPE = "Rampe";

	public ConveyorRamp() {
		super(0, 0, Conveyor.getRastersize(), Conveyor.getRastersize());
	}
	
	public ConveyorRamp(int x, int y) {
		super(x, y, Conveyor.getRastersize(), Conveyor.getRastersize());
	}

	@Override
	protected Canvas createCanvas(Canvas canvas) {
		Context2d context = canvas.getContext2d();
		
		context.setFillStyle(CssColor.make("red"));
		context.fillRect(0, 0, getWidth(), getHeight());
		context.fill();
		
		return canvas;
	}

	@Override
	public String getType() {
		return ConveyorRamp.TYPE;
	}
}
