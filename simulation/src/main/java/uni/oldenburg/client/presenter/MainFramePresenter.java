package uni.oldenburg.client.presenter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import uni.oldenburg.client.service.SimulationServiceAsync;
import uni.oldenburg.client.view.DialogBoxScenarioSelection;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.ConveyorRamp;
import uni.oldenburg.shared.model.ConveyorVehicle;
import uni.oldenburg.shared.model.Job;
import uni.oldenburg.shared.model.Szenario;

import com.google.gwt.canvas.client.Canvas;
import com.google.gwt.canvas.dom.client.Context2d;
import com.google.gwt.canvas.dom.client.CssColor;
import com.google.gwt.dom.client.NativeEvent;
import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.event.dom.client.HasClickHandlers;
import com.google.gwt.event.dom.client.KeyCodes;
import com.google.gwt.event.dom.client.KeyUpEvent;
import com.google.gwt.event.dom.client.KeyUpHandler;
import com.google.gwt.event.dom.client.MouseMoveEvent;
import com.google.gwt.event.dom.client.MouseMoveHandler;
import com.google.gwt.event.dom.client.MouseUpEvent;
import com.google.gwt.event.dom.client.MouseUpHandler;
import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.cellview.client.CellTable;
import com.google.gwt.user.cellview.client.TextColumn;
import com.google.gwt.user.client.Command;
import com.google.gwt.user.client.Random;
import com.google.gwt.user.client.Window;
import com.google.gwt.user.client.rpc.AsyncCallback;
import com.google.gwt.user.client.ui.MenuBar;
import com.google.gwt.user.client.ui.Widget;
import com.google.gwt.view.client.AsyncDataProvider;
import com.google.gwt.view.client.HasData;

public class MainFramePresenter extends Presenter {
	private final IDisplay display;
	private Szenario currentSzenario;
	Conveyor dropableConveyor;

	private static final List<Job> JOBS = Arrays.asList(
			new Job(1, "XY", Job.OUTGOING, 42),
			new Job(2, "zt", Job.OUTGOING, 73),
			new Job(3, "Patata", Job.INCOMING, 128),
			new Job(4, "UO", Job.OUTGOING, 1337)
	);

	public interface IDisplay {
		CellTable<Job> getJobTable();
		HasClickHandlers getStrategiesButton();
		HasClickHandlers getVirtualHybridButton();
		HasClickHandlers getConveyorRampButton();
		HasClickHandlers getConveyorVehicleButton();
		MenuBar getMenuBar();
		MenuBar getSimulationMenuBar();
		MenuBar getEditMenuBar();
		Canvas getCanvas();
	}

	public MainFramePresenter(SimulationServiceAsync rpcService, HandlerManager eventBus, IDisplay view) {
		super(rpcService, eventBus);
		this.display = view;
		this.currentSzenario = new Szenario();
		this.dropableConveyor = null;
	}

	public Widget getDisplay() {
		return (Widget) display;
	}
	
	private void addCanvasListener() {
		display.getCanvas().addMouseMoveHandler(new MouseMoveHandler() {
			public void onMouseMove(MouseMoveEvent event) {				
				Conveyor myConveyor = MainFramePresenter.this.dropableConveyor;
				
				if (myConveyor == null)
					return;
				
				myConveyor.setPosition(event.getX(), event.getY());				
				
				loadSzenario(MainFramePresenter.this.currentSzenario);
				drawConveyor(myConveyor);
				
				display.getCanvas().setFocus(true);				
			}
		});
		
		display.getCanvas().addMouseUpHandler(new MouseUpHandler() {
			public void onMouseUp(MouseUpEvent event) {				
				if (event.getNativeButton() == NativeEvent.BUTTON_LEFT) {
					Conveyor myConveyor = MainFramePresenter.this.dropableConveyor;
					
					if (myConveyor != null) {
						MainFramePresenter.this.currentSzenario.addConveyor(myConveyor);
						MainFramePresenter.this.dropableConveyor = null;
						loadSzenario(MainFramePresenter.this.currentSzenario);
					}
				}
			}
		});
		
		display.getCanvas().addKeyUpHandler(new KeyUpHandler() {
			public void onKeyUp(KeyUpEvent event) {
				if (event.getNativeKeyCode() == KeyCodes.KEY_ESCAPE) {
					MainFramePresenter.this.dropableConveyor = null;					
					loadSzenario(MainFramePresenter.this.currentSzenario);
				}
			}
		});
	}

	private void addStrategiesButtonListener() {
		display.getStrategiesButton().addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				// TODO
			}
		});
	}

	private void addVirtualHybridButtonListener() {
		display.getVirtualHybridButton().addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				// TODO
			}
		});
	}

	private void addConveyorRampButtonListener() {
		display.getConveyorRampButton().addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				MainFramePresenter.this.dropableConveyor = new ConveyorRamp();
			}
		});
	}

	private void addConveyorVehicleButtonListener() {
		display.getConveyorVehicleButton().addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				MainFramePresenter.this.dropableConveyor = new ConveyorVehicle();
			}
		});
	}

	private void setupJobTable() {
		TextColumn<Job> jobNumberColumn = new TextColumn<Job>() {
			@Override
			public String getValue(Job object) {
				return "" + object.getJobNumber();
			}
		};

		TextColumn<Job> jobColumn = new TextColumn<Job>() {
			@Override
			public String getValue(Job object) {
				return object.getJob();
			}
		};

		display.getJobTable().addColumn(jobNumberColumn, "Job Number");
		display.getJobTable().addColumn(jobColumn, "Job");

		AsyncDataProvider<Job> provider = new AsyncDataProvider<Job>() {
			@Override
			protected void onRangeChanged(HasData<Job> display) {
				int start = display.getVisibleRange().getStart();
				int end = start + display.getVisibleRange().getLength();
				end = end >= JOBS.size() ? JOBS.size() : end;
				List<Job> sub = JOBS.subList(start, end);
				updateRowData(start, sub);
			}
		};
		provider.addDataDisplay(display.getJobTable());
		provider.updateRowCount(JOBS.size(), true);
	}

	private void drawConveyor(Conveyor myConveyor) {
		Context2d context = display.getCanvas().getContext2d();
		context.drawImage(myConveyor.getCanvasElement(), myConveyor.getX(), myConveyor.getY());
	}

	@SuppressWarnings("unused")
	private void generateConveyor() {
		for (int i = 0; i < 5; ++i) {
			drawConveyor(new ConveyorRamp(Random.nextInt(780),
					Random.nextInt(460)));
		}

		for (int i = 0; i < 5; ++i) {
			drawConveyor(new ConveyorVehicle(Random.nextInt(780),
					Random.nextInt(460)));
		}
	}
	
	public void clearCanvas() {
		Context2d context = display.getCanvas().getContext2d();
		context.setFillStyle(CssColor.make(255, 255, 255));
		context.fillRect(0,  0, display.getCanvas().getCoordinateSpaceWidth(), display.getCanvas().getCoordinateSpaceHeight());
		context.fill();
	}
	
	public void loadSzenario(Szenario szenario) {
		clearCanvas();		
		
		if (szenario == null)
			return;
		
		List<Conveyor> lstConveyor = szenario.getConveyorList();
		
		for (Conveyor myConveyor : lstConveyor) {
			drawConveyor(myConveyor);
		}
	}

	public void loadSzenario(String name) {
		((SimulationServiceAsync) rpcService).loadSzenario(name, new AsyncCallback<Szenario>() {
			public void onFailure(Throwable arg0) {
				Window.alert(arg0.getLocalizedMessage());
			}

			public void onSuccess(Szenario szenario) {
				MainFramePresenter.this.currentSzenario = szenario;
				loadSzenario(szenario);
			}
		});
	}

	// Gets the Scenario Titles From Server and displays it in a Dialog
	public void getScenarioTitlesFromServerAndShow() {
		((SimulationServiceAsync) rpcService).getScenarioTitles(new AsyncCallback<ArrayList<String>>() {
			public void onFailure(Throwable arg0) {
				Window.alert(arg0.getLocalizedMessage());
			}

			public void onSuccess(ArrayList<String> result) {
				DialogBoxScenarioSelection dialog = new DialogBoxScenarioSelection(result, MainFramePresenter.this);
				dialog.show();
			}
		});
	}

	public void bind() {
		this.initializeMenuBars();
		this.addStrategiesButtonListener();
		this.addVirtualHybridButtonListener();
		this.addConveyorRampButtonListener();
		this.addConveyorVehicleButtonListener();
		this.addCanvasListener();
		this.setupJobTable();
	}

	// Method initializes the Menubar. This is done here, because it is
	// necessary to embed Client-Server Communication
	// into the Execute-methods of the Menubaritems. It would be ugly to insert
	// the RPC-Service into a View
	private void initializeMenuBars() {
		// --- menu bar ---

		// file menu

		this.display.getSimulationMenuBar().addItem("Load Scenario", new Command() {
			public void execute() {
				getScenarioTitlesFromServerAndShow();
			}
		});

		this.display.getSimulationMenuBar().addItem("Save", new Command() {
			public void execute() {

			}
		});
		this.display.getSimulationMenuBar().addItem("Save As", new Command() {
			public void execute() {

			}
		});
		
		this.display.getSimulationMenuBar().addSeparator();		
		
		this.display.getSimulationMenuBar().addItem("Simulation Settings", new Command() {
			public void execute() {

			}
		});
		
		this.display.getSimulationMenuBar().addSeparator();				
		
		this.display.getSimulationMenuBar().addItem("Start/Stop Simulation", new Command() {
			public void execute() {

			}
		});		

		// edit menu
		this.display.getEditMenuBar().addItem("Define Job Set", new Command() {
			public void execute() {

			}
		});
		
		this.display.getEditMenuBar().addItem("Edit Jobs", new Command() {
			public void execute() {

			}
		});

		// menu bar

		this.display.getMenuBar().addItem("Simulation", this.display.getSimulationMenuBar());
		this.display.getMenuBar().addSeparator();
		this.display.getMenuBar().addItem("Edit", this.display.getEditMenuBar());
	}

}
