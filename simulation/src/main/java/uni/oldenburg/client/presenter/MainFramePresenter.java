package uni.oldenburg.client.presenter;

import java.util.ArrayList;
import java.util.List;

import uni.oldenburg.client.service.ServiceAsync;
import uni.oldenburg.client.service.SimulationServiceAsync;
import uni.oldenburg.client.view.DialogBoxJoblistSelection;
import uni.oldenburg.client.view.DialogBoxOverwrite;
import uni.oldenburg.client.view.DialogBoxOverwriteJoblist;
import uni.oldenburg.client.view.DialogBoxSaveAs;
import uni.oldenburg.client.view.DialogBoxSaveAsJoblist;
import uni.oldenburg.client.view.DialogBoxScenarioSelection;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.ConveyorRamp;
import uni.oldenburg.shared.model.ConveyorVehicle;
import uni.oldenburg.shared.model.ConveyorWall;
import uni.oldenburg.shared.model.Job;
import uni.oldenburg.shared.model.JobList;
import uni.oldenburg.shared.model.Szenario;

import com.google.gwt.canvas.client.Canvas;
import com.google.gwt.canvas.dom.client.Context2d;
import com.google.gwt.canvas.dom.client.CssColor;
import com.google.gwt.cell.client.ActionCell;
import com.google.gwt.dom.client.NativeEvent;
import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.event.dom.client.ContextMenuEvent;
import com.google.gwt.event.dom.client.ContextMenuHandler;
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
import com.google.gwt.user.cellview.client.Column;
import com.google.gwt.user.cellview.client.TextColumn;
import com.google.gwt.user.client.Command;
import com.google.gwt.user.client.Random;
import com.google.gwt.user.client.Window;
import com.google.gwt.user.client.rpc.AsyncCallback;
import com.google.gwt.user.client.ui.HasText;
import com.google.gwt.user.client.ui.Label;
import com.google.gwt.user.client.ui.MenuBar;
import com.google.gwt.user.client.ui.Widget;
import com.google.gwt.view.client.AsyncDataProvider;
import com.google.gwt.view.client.HasData;

public class MainFramePresenter extends Presenter {
	private final IDisplay display;
	private Szenario currentSzenario;
	Conveyor dropableConveyor;
	JobList lstJobs = new JobList();

	public interface IDisplay {
		CellTable<Job> getJobTable();
		
		HasClickHandlers getAddJobsButton();
		HasClickHandlers getStrategiesButton();
		HasClickHandlers getConveyorRampButton();
		HasClickHandlers getConveyorVehicleButton();
		HasClickHandlers getConveyorWallButton();
		
		HasText	getJobCount();

		MenuBar getMenuBar();
		MenuBar getSimulationMenuBar();
		MenuBar getEditMenuBar();
		Label getLabelUserName();
		Canvas getCanvas();
		
		void log(String log);		
	}

	public MainFramePresenter(SimulationServiceAsync rpcService,
			HandlerManager eventBus, IDisplay view) {
		super(rpcService, eventBus);
		this.display = view;
		this.currentSzenario = new Szenario();
		this.dropableConveyor = null;
	}

	public Widget getDisplay() {
		return (Widget) display;

	}

	/**
	 * Method gets UserName from Server and writes it into the Label
	 * 
	 * @author Nagi
	 */
	private void setLabelUserName() {

		// Get the Username from Server
		((SimulationServiceAsync) rpcService).getUserName(new AsyncCallback<String>() {
					public void onFailure(Throwable arg0) {
						Window.alert(arg0.getLocalizedMessage());
					}

					public void onSuccess(String result) {
						// Write the String into the Label
						((MainFramePresenter.IDisplay) MainFramePresenter.this.getDisplay())
							.getLabelUserName().setText("Eingeloggt als: " + result);
					}
				});
	}

	/**
	 * setup listener of canvas object for managing drag & drop handling
	 * 
	 * @author Matthias
	 */
	private void addCanvasListener() {
		/**
		 * disable default context menu in canvas
		 * 
		 * @author Matthias
		 */
		display.getCanvas().addDomHandler(new ContextMenuHandler() {
			public void onContextMenu(ContextMenuEvent event) {
				event.preventDefault();
				event.stopPropagation();
			}
		}, ContextMenuEvent.getType());

		/**
		 * show movement of drag&drop-able object
		 * 
		 * @author Matthias
		 */
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

		/**
		 * drag, drop and rotate objects on canvas
		 * 
		 * @author Matthias
		 */
		display.getCanvas().addMouseUpHandler(new MouseUpHandler() {
			public void onMouseUp(MouseUpEvent event) {
				Conveyor myConveyor = MainFramePresenter.this.dropableConveyor;

				// conveyor drag & drop
				if (event.getNativeButton() == NativeEvent.BUTTON_LEFT) {
					// add conveyor
					if (myConveyor != null) {
						// when spot available
						if (isSpotAvailable(event.getX(), event.getY())) {
							MainFramePresenter.this.currentSzenario
									.addConveyor(myConveyor);
							MainFramePresenter.this.dropableConveyor = null;
							loadSzenario(MainFramePresenter.this.currentSzenario);
						}
					} else {
						// grab & move conveyor
						grabConveyor(event.getX(), event.getY());
					}
				} else if (event.getNativeButton() == NativeEvent.BUTTON_RIGHT) {
					// rotate conveyors
					if (myConveyor == null)
						return;

					if (myConveyor.getType().compareTo(ConveyorRamp.TYPE) == 0 || 
						myConveyor.getType().compareTo(ConveyorVehicle.TYPE) == 0) {
						myConveyor.rotateClockwise();

						loadSzenario(MainFramePresenter.this.currentSzenario);
						drawConveyor(myConveyor);

						display.getCanvas().setFocus(true);
					}
				}
			}
		});

		/**
		 * keyboard handling for canvas to delete conveyor
		 * 
		 * @author Matthias
		 */
		display.getCanvas().addKeyUpHandler(new KeyUpHandler() {
			public void onKeyUp(KeyUpEvent event) {
				if (event.getNativeKeyCode() == KeyCodes.KEY_ESCAPE) {
					MainFramePresenter.this.dropableConveyor = null;
					loadSzenario(MainFramePresenter.this.currentSzenario);
				}
			}
		});
	}

	/**
	 * add random jobs
	 * 
	 * @author Matthias
	 */
	private void addAddJobsButtonListener() {
		display.getAddJobsButton().addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {

				String jobCount = MainFramePresenter.this.display.getJobCount().getText();
				
				if (!jobCount.matches("^\\d+$")) {
					Window.alert("Fehler: Es wurde keine Zahl eingetragen!");
				}
				else {
					lstJobs.addRandomJobs(Integer.parseInt(jobCount));
					setupJobTable();
				}
				
				MainFramePresenter.this.display.getJobCount().setText("1");

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

	/**
	 * select ramp as drag/drop-able conveyor
	 * 
	 * @author Matthias
	 */
	private void addConveyorRampButtonListener() {
		display.getConveyorRampButton().addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				MainFramePresenter.this.dropableConveyor = new ConveyorRamp();
			}
		});
	}

	/**
	 * select vehicle as drag/drop-able conveyor
	 * 
	 * @author Matthias
	 */
	private void addConveyorVehicleButtonListener() {
		display.getConveyorVehicleButton().addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				MainFramePresenter.this.dropableConveyor = new ConveyorVehicle();
			}
		});
	}

	/**
	 * select wall as drag/drop-able conveyor
	 * 
	 * @author Matthias
	 */
	private void addConveyorWallButtonListener() {
		display.getConveyorWallButton().addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				MainFramePresenter.this.dropableConveyor = new ConveyorWall();
			}
		});
	}
	
	/**
	 * @author Christopher Matthias
	 */
	private void setupJobTable() {		
		TextColumn<Job> timestampColumn = new TextColumn<Job>() {
			@Override
			public String getValue(Job object) {
				return "" + object.getTimestamp();
			}
		};
		TextColumn<Job> packageColumn = new TextColumn<Job>() {
			@Override
			public String getValue(Job object) {
				return "" + object.getPackageId();
			}
		};
		TextColumn<Job> destinationColumn = new TextColumn<Job>() {
			@Override
			public String getValue(Job object) {
				if (object.getDestinationId() == 0) {
					return "Lager";
				}
				return "" + object.getDestinationId();
			}
		};
		
		while(display.getJobTable().getColumnCount() > 0) {
			display.getJobTable().removeColumn(0);
		}
		display.getJobTable().addColumn(timestampColumn, "Zeit");
		display.getJobTable().addColumn(packageColumn, "Paket");
		display.getJobTable().addColumn(destinationColumn, "Ziel");
		
		// add clickable action button column
		ActionCell<Job> actionDeleteCell = new ActionCell<Job>("-", new ActionCell.Delegate<Job>() {
			public void execute(Job job) {
				List<Job> lstJobDeleteable = new ArrayList<Job>();
				// mark selected job as deleteable
				lstJobDeleteable.add(job);
				
				if (job.getType() == Job.INCOMING) {
					// delete linked outgoing job too
					for (Job jobEntry : MainFramePresenter.this.lstJobs.getJoblist()) {
						if (jobEntry.getPackageId() == job.getPackageId()) {
							lstJobDeleteable.add(jobEntry);
						}
					}	
				}
				
				// delete in actual joblist
				for (Job jobEntry : lstJobDeleteable) {
					MainFramePresenter.this.lstJobs.removeJob(jobEntry);	
				}
				
				// update joblist
				MainFramePresenter.this.setupJobTable();
			}
		});
		
		display.getJobTable().addColumn(new Column<Job, Job>(actionDeleteCell) {
			@Override
			public Job getValue(Job job) {
				return job;
			}
		}, "Aktion");
		// ----------------------------------

		final JobList jobList = this.lstJobs;

		AsyncDataProvider<Job> provider = new AsyncDataProvider<Job>() {
			@Override
			protected void onRangeChanged(HasData<Job> display) {
				int start = display.getVisibleRange().getStart();
				int end = start + display.getVisibleRange().getLength();
				end = end >= jobList.size() ? jobList.size() : end;
				List<Job> sub = jobList.subList(start, end);
				updateRowData(start, sub);
			}
		};
		provider.addDataDisplay(display.getJobTable());
		provider.updateRowCount(jobList.size(), true);
	}

	/**
	 * draw conveyor von main canvas
	 * 
	 * @author Matthias
	 */
	private void drawConveyor(Conveyor myConveyor) {
		Context2d context = display.getCanvas().getContext2d();
		context.drawImage(myConveyor.getCanvasElement(), myConveyor.getX(),
				myConveyor.getY());
	}

	/**
	 * draw conveyors on random positions
	 * 
	 * @author Matthias
	 */
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

	/**
	 * check if spot is taken by a conveyor
	 * 
	 * @author Matthias
	 */
	public boolean isSpotAvailable(int x, int y) {
		return (findConveyor(x, y) == null);
	}

	/**
	 * find conveyor on given location
	 * 
	 * @author Matthias
	 */
	public Conveyor findConveyor(int x, int y) {
		Conveyor myConveyor = null;

		List<Conveyor> lstConveyor = this.currentSzenario.getConveyorList();

		for (Conveyor cvEntry : lstConveyor) {
			if (x < cvEntry.getX())
				continue;
			if (y < cvEntry.getY())
				continue;
			if (x > (cvEntry.getX() + cvEntry.getWidth()))
				continue;
			if (y > (cvEntry.getY() + cvEntry.getHeight()))
				continue;

			myConveyor = cvEntry;
			break;
		}

		return myConveyor;
	}

	/**
	 * grab conveyor at given location
	 * 
	 * @author Matthias
	 */
	public void grabConveyor(int x, int y) {
		Conveyor myConveyor = findConveyor(x, y);

		if (myConveyor != null)
			this.currentSzenario.removeConveyor(myConveyor);

		this.dropableConveyor = myConveyor;
	}

	/**
	 * clear the main canvas for redrawing
	 * 
	 * @author Matthias
	 */
	public void clearCanvas() {
		Context2d context = display.getCanvas().getContext2d();
		context.setFillStyle(CssColor.make(255, 255, 255));
		context.fillRect(0, 0, display.getCanvas().getCoordinateSpaceWidth(),
				display.getCanvas().getCoordinateSpaceHeight());
		context.fill();

	}

	/**
	 * draw current szenario
	 * 
	 * @author Matthias
	 */
	public void loadSzenario(Szenario szenario) {
		clearCanvas();

		if (szenario == null)
			return;

		List<Conveyor> lstConveyor = szenario.getConveyorList();

		for (Conveyor myConveyor : lstConveyor) {
			drawConveyor(myConveyor);
		}
	}

	/**
	 * load szenario from database and draw it
	 * 
	 * @author Matthias
	 */
	public void loadSzenario(String name) {
		((SimulationServiceAsync) rpcService).loadSzenario(name,
				new AsyncCallback<Szenario>() {
					public void onFailure(Throwable arg0) {
						Window.alert(arg0.getLocalizedMessage());
					}

					public void onSuccess(Szenario szenario) {
						MainFramePresenter.this.currentSzenario = szenario;
						loadSzenario(szenario);
					}
				});
	}

	/**
	 * Gets the Scenario Titles From Server and displays it in a Dialog
	 * 
	 * @author Raschid
	 */
	public void getScenarioTitlesFromServerAndShow() {
		((SimulationServiceAsync) rpcService)
				.getScenarioTitles(new AsyncCallback<ArrayList<String>>() {
					public void onFailure(Throwable arg0) {
						Window.alert(arg0.getLocalizedMessage());
					}

					public void onSuccess(ArrayList<String> result) {
						DialogBoxScenarioSelection dialog = new DialogBoxScenarioSelection(
								result, MainFramePresenter.this);
						dialog.show();
					}
				});
	}

	/**
	 * Method checks if Szenario exists and then decides wether to save or to
	 * open the Save As Dialog
	 * 
	 * @author Raschid
	 */
	public void trySaveSzenario(Szenario szenario) {

		// Check if Szenario already exists
		((SimulationServiceAsync) rpcService).checkIfTitleExists(
				szenario.getTitle(), new AsyncCallback<Boolean>() {
					public void onFailure(Throwable caught) {
						Window.alert("Communication Problem");
					}

					public void onSuccess(Boolean result) {
						// If Szenario exists open a popup, where the user can
						// be asked if he wants to overwritte or not
						if (result) {
							// Show the popup so the user can select if he wants
							// to overwrite or not
							DialogBoxOverwrite dialog = new DialogBoxOverwrite(
									MainFramePresenter.this);
							dialog.show();
						} else {
							// Open Save as Dialog
							DialogBoxSaveAs dialog = new DialogBoxSaveAs(
									MainFramePresenter.this);
							dialog.show();
						}
					}
				});
	}

	/**
	 * Method sends the Szenario to the server in order to write it into the
	 * database. The operation String decides wether an Update or an Insert
	 * should be made
	 * 
	 * @author Raschid
	 */
	public void sendSzenarioToServer(Szenario szenario) {

		// Send Szenario to Server
		((SimulationServiceAsync) rpcService).saveSzenario(szenario,
				new AsyncCallback<Void>() {

					public void onFailure(Throwable caught) {
						Window.alert("Communication Problem, Szenario was not saved");

					}

					public void onSuccess(Void result) {
						Window.alert("Szenario was successfully saved");

					}

				});
	}
	
	/**
	 * draw current joblist
	 * 
	 * @author Raschid
	 */
	public void loadJoblist() {
		this.setupJobTable();
	}

	/**
	 * load joblist from database and draw it
	 * 
	 * @author Raschid
	 */
	public void loadJoblist(String name) {
		((SimulationServiceAsync) rpcService).loadJoblist(name,
				new AsyncCallback<JobList>() {
					public void onFailure(Throwable arg0) {
						Window.alert(arg0.getLocalizedMessage());
					}

					public void onSuccess(JobList list) {
						MainFramePresenter.this.lstJobs = list;
						loadJoblist();
					}
				});
	}

	/**
	 * Gets the Joblist Titles From Server and displays it in a Dialog
	 * 
	 * @author Raschid
	 */
	public void getJoblistTitlesFromServerAndShow() {
		((SimulationServiceAsync) rpcService)
				.getJoblistTitles(new AsyncCallback<ArrayList<String>>() {
					public void onFailure(Throwable arg0) {
						Window.alert(arg0.getLocalizedMessage());
					}

					public void onSuccess(ArrayList<String> result) {
						DialogBoxJoblistSelection dialog = new DialogBoxJoblistSelection(
								result, MainFramePresenter.this);
						dialog.show();
					}
				});
	}

	/**
	 * Method checks if Joblist exists and then decides wether to save or to
	 * open the Save As Dialog
	 * 
	 * @author Raschid
	 */
	public void trySaveJoblist(JobList joblist) {
		// Check if Szenario already exists
		((SimulationServiceAsync) rpcService).checkIfJobListExists(
				joblist.getName(), new AsyncCallback<Boolean>() {
					public void onFailure(Throwable caught) {
						Window.alert("Communication Problem");
					}

					public void onSuccess(Boolean result) {
						// If Joblist exists open a popup, where the user can
						// be asked if he wants to overwrite or not
						if (result) {
							// Show the popup so the user can select if he wants
							// to overwrite or not
							DialogBoxOverwriteJoblist dialog = new DialogBoxOverwriteJoblist(
									MainFramePresenter.this);
							dialog.show();
						} else {
							// Open Save as Dialog
							DialogBoxSaveAsJoblist dialog = new DialogBoxSaveAsJoblist(
									MainFramePresenter.this);
							dialog.show();
						}
					}
				});
	}

	/**
	 * Method sends the Joblist to the server in order to write it into the
	 * database. The operation String decides wether an Update or an Insert
	 * should be made
	 * 
	 * @author Raschid
	 */
	public void sendJoblistToServer(JobList list) {

		// Send Szenario to Server
		((SimulationServiceAsync) rpcService).saveJoblist(list,
				new AsyncCallback<Void>() {

					public void onFailure(Throwable caught) {
						Window.alert("Communication Problem, Joblist was not saved");

					}

					public void onSuccess(Void result) {
						Window.alert("Joblist was successfully saved");

					}

				});
	}

	public void bind() {
		this.initializeMenuBars();
		this.addAddJobsButtonListener();
		this.addStrategiesButtonListener();
		this.addConveyorRampButtonListener();
		this.addConveyorVehicleButtonListener();
		this.addConveyorWallButtonListener();
		this.addCanvasListener();
		this.setupJobTable();
		this.setLabelUserName();

	}

	/**
	 * Method initializes the Menubar. This is done here, because it is
	 * necessary to embed Client-Server Communication into the Execute-methods
	 * of the Menubaritems. It would be ugly to insert the RPC-Service into a
	 * View
	 * 
	 * @author Raschid
	 */
	private void initializeMenuBars() {
		// --- menu bar ---

		// file menu

		this.display.getSimulationMenuBar().addItem("Laden", new Command() {
			public void execute() {
				getScenarioTitlesFromServerAndShow();
			}
		});

		this.display.getSimulationMenuBar().addItem("Speichern", new Command() {
			public void execute() {
				trySaveSzenario(MainFramePresenter.this.getActualSzenario());
			}
		});
		this.display.getSimulationMenuBar().addItem("Speichern unter...",
				new Command() {
					public void execute() {
						// Open Save as Dialog
						DialogBoxSaveAs dialog = new DialogBoxSaveAs(
								MainFramePresenter.this);
						dialog.show();
					}
				});

		this.display.getSimulationMenuBar().addSeparator();

		this.display.getSimulationMenuBar().addItem("Einstellungen",
				new Command() {
					public void execute() {

					}
				});

		this.display.getSimulationMenuBar().addSeparator();

		this.display.getSimulationMenuBar().addItem("Starten/Anhalten",
				new Command() {
					public void execute() {

					}
				});

		// edit menu
		this.display.getEditMenuBar().addItem("Laden",
				new Command() {
					public void execute() {
						getJoblistTitlesFromServerAndShow();
					}
				});

		this.display.getEditMenuBar().addItem(
				"Speichern", new Command() {
					public void execute() {
						trySaveJoblist(MainFramePresenter.this.getActualJoblist());
					}
				});
		
		this.display.getEditMenuBar().addItem(
				"Speichern unter...", new Command() {
					public void execute() {
						// Open Save as Dialog
						DialogBoxSaveAsJoblist dialog = new DialogBoxSaveAsJoblist(
								MainFramePresenter.this);
						dialog.show();
					}
				});

		// menu bar

		this.display.getMenuBar().addItem("Simulation",
				this.display.getSimulationMenuBar());
		this.display.getMenuBar().addSeparator();

		this.display.getMenuBar().addItem("Auftragsliste",
				this.display.getEditMenuBar());
	}

	public Szenario getActualSzenario() {
		return this.currentSzenario;
	}
	
	public JobList getActualJoblist() {
		return this.lstJobs;
	}

	public ServiceAsync getService() {
		return this.rpcService;
	}
}
