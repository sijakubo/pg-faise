package uni.oldenburg.client.view;

import uni.oldenburg.client.presenter.MainFramePresenter;
import uni.oldenburg.shared.model.Job;

import com.google.gwt.canvas.client.Canvas;
import com.google.gwt.dom.client.Style.BorderStyle;
import com.google.gwt.dom.client.Style.Unit;
import com.google.gwt.event.dom.client.HasClickHandlers;
import com.google.gwt.user.cellview.client.CellTable;
import com.google.gwt.user.cellview.client.SimplePager;
import com.google.gwt.user.client.ui.Button;
import com.google.gwt.user.client.ui.Composite;
import com.google.gwt.user.client.ui.HasText;
import com.google.gwt.user.client.ui.Label;
import com.google.gwt.user.client.ui.MenuBar;
import com.google.gwt.user.client.ui.HorizontalPanel;
import com.google.gwt.user.client.ui.Panel;
import com.google.gwt.user.client.ui.TextArea;
import com.google.gwt.user.client.ui.TextBox;
import com.google.gwt.user.client.ui.VerticalPanel;

public class MainFrameView extends Composite implements MainFramePresenter.IDisplay {
	private MenuBar			menuBar;
	private MenuBar			simMenuBar;
	private MenuBar			jobMenuBar;
	
	private CellTable<Job>	ctJobTable = new CellTable<Job>();
	private Button			btnAddJobs;
	private Button         	btnStrategies;
	
	// conveyor menu
	private VerticalPanel 	vpConveyor;
	
	private Label			lblConveyor;
	private Button			btnConveyorRamp;
	private Button			btnConveyorVehicle;
	private Button			btnConveyorWall;
	// -------------
	private TextBox 		txtJobCount;
	private TextArea 		txtDebug;
	
	private Canvas			canvas;
	
	public static final int canvasWidth  = 800;
	public static final int canvasHeight = 480;
	
	private static int logLineCount = 0;
	
	public MainFrameView() {
		VerticalPanel   vpMainFrame = new VerticalPanel();
		HorizontalPanel hpSubFrame = new HorizontalPanel();
		VerticalPanel   vpLeftFrame = new VerticalPanel();
		vpLeftFrame.addStyleName("left_panel");
			
		//--- menu bar ---
		
		//simulation menu
		
		simMenuBar = new MenuBar(true);
		simMenuBar.setAnimationEnabled(true);
		
		//edit menu
		
		jobMenuBar = new MenuBar(true);
		jobMenuBar.setAnimationEnabled(true);
		
		//menu bar
		
		menuBar = new MenuBar();
	
		//--- left panel ---

		//table
		ctJobTable.setStyleName("jobtable");
		
		ctJobTable.setPageSize(10);
	    vpLeftFrame.add(ctJobTable);

	    SimplePager pager = new SimplePager();
	    pager.setDisplay(ctJobTable);

	    VerticalPanel vp = new VerticalPanel();
	    vp.add(ctJobTable);
	    vp.add(pager);
	    vpLeftFrame.add(vp);
	    
	    // generate random job entries control
	    HorizontalPanel hpGenerateJobs = new HorizontalPanel();
	    hpGenerateJobs.setSpacing(10);
	    hpGenerateJobs.setVerticalAlignment(HorizontalPanel.ALIGN_MIDDLE);
	    
	    hpGenerateJobs.add(new Label("Auftr" + (char)228 + "ge: "));
	    
	    txtJobCount = new TextBox();
	    txtJobCount.setText("1");
	    txtJobCount.setMaxLength(4);
	    txtJobCount.setWidth("30px");
	    hpGenerateJobs.add(txtJobCount);

	    btnAddJobs = new Button("hinzuf" + (char)252 + "gen");
	    hpGenerateJobs.add(btnAddJobs);
	    
	    vpLeftFrame.add(hpGenerateJobs);
		
		//button for changing pathfinding strategy
		
		btnStrategies = new Button("Strategien");
		btnStrategies.setText("Strategien");
		vpLeftFrame.add(btnStrategies);
		
		// conveyor list
		vpConveyor = new VerticalPanel();
		vpConveyor.addStyleName("conveyor_panel");
		
		lblConveyor = new Label("Stetigf" + (char)246 + "rderer");
		lblConveyor.addStyleName("conveyor_label");
		btnConveyorRamp = new Button("Rampe");
		btnConveyorRamp.addStyleName("conveyor_ramp");
		btnConveyorVehicle = new Button("Fahrzeug");
		btnConveyorVehicle.addStyleName("conveyor_vehicle");
		btnConveyorWall = new Button("Wand");
		btnConveyorWall.addStyleName("conveyor_wall");

		vpConveyor.add(lblConveyor);
		vpConveyor.add(btnConveyorRamp);
		vpConveyor.add(btnConveyorVehicle);
		vpConveyor.add(btnConveyorWall);
		vpLeftFrame.add(vpConveyor);
		
		
      // -------------
		
		hpSubFrame.add(vpLeftFrame);
		
		// create canvas object
		canvas = Canvas.createIfSupported();
		canvas.setWidth(canvasWidth + "px");
		canvas.setHeight(canvasHeight + "px");
		canvas.setCoordinateSpaceWidth(canvasWidth);
		canvas.setCoordinateSpaceHeight(canvasHeight);
		canvas.getElement().getStyle().setBorderStyle(BorderStyle.SOLID);
		canvas.getElement().getStyle().setBorderWidth(1,Unit.PX);
		hpSubFrame.add(canvas);
		
		txtDebug = new TextArea();
		txtDebug.setWidth("100%");
		txtDebug.setHeight("100px");
		
		vpMainFrame.add(menuBar);
		vpMainFrame.add(hpSubFrame);
		vpMainFrame.add(txtDebug);
		
		initWidget(vpMainFrame);
	}

    public HasClickHandlers getStrategiesButton() {
        return btnStrategies;
    }

    public CellTable<Job> getJobTable() {
        return ctJobTable;
    }

    public HasClickHandlers getAddJobsButton() {
        return btnAddJobs;
    }

	public Canvas getCanvas() {
		return canvas;
	}

	public HasClickHandlers getConveyorRampButton() {
		return btnConveyorRamp;
	}

	public HasClickHandlers getConveyorVehicleButton() {
		return btnConveyorVehicle;
	}

	public MenuBar getMenuBar() {
		return this.menuBar;
	}

	public MenuBar getSimulationMenuBar() {
		return this.simMenuBar;
	}

	public MenuBar getJobMenuBar() {
		return this.jobMenuBar;
	}

	public HasClickHandlers getConveyorWallButton() {
		return this.btnConveyorWall;
	}

	public void log(String log) {
		this.txtDebug.setText(++logLineCount + ": " + log + "\n" + this.txtDebug.getText());
	}

	public HasText getJobCount() {
		return txtJobCount;
	}

	public Panel getConveyorPanel() {
		return vpConveyor;
	}
}