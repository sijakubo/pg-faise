package uni.oldenburg.client.view;

import java.util.Arrays;
import java.util.List;

import uni.oldenburg.client.presenter.MainFramePresenter;

import com.google.gwt.canvas.client.Canvas;
import com.google.gwt.user.cellview.client.CellTable;
import com.google.gwt.user.cellview.client.SimplePager;
import com.google.gwt.user.cellview.client.TextColumn;
import com.google.gwt.user.client.Command;
import com.google.gwt.user.client.ui.Button;
import com.google.gwt.user.client.ui.Composite;
import com.google.gwt.user.client.ui.MenuBar;
import com.google.gwt.user.client.ui.HorizontalPanel;
import com.google.gwt.user.client.ui.ToggleButton;
import com.google.gwt.user.client.ui.VerticalPanel;
import com.google.gwt.view.client.AsyncDataProvider;
import com.google.gwt.view.client.HasData;

public class MainFrameView extends Composite implements MainFramePresenter.IDisplay {
	
	private MenuBar        menuBar;	
	private MenuBar        fileMenuBar;
	private MenuBar        editMenuBar;
	private MenuBar        viewMenuBar;
	
	private CellTable<Job> jobTable = new CellTable<Job>();
	private Button         btStrategies;
	private ToggleButton   tbVirtualHybridSwitch;
	
	private Canvas       canvas;
	
	private static final List<Job> JOBS = Arrays.asList(
		      new Job(1, "XY"),
		      new Job(2, "zt"),
		      new Job(3, "Patata"),
		      new Job(4, "UO"));
	
	public MainFrameView() {

		VerticalPanel   vpMainFrame = new VerticalPanel();
		HorizontalPanel hpSubFrame = new HorizontalPanel();
		VerticalPanel   vpLeftFrame = new VerticalPanel();
			
		//--- menu bar ---
		
		//file menu
		
		fileMenuBar = new MenuBar(true);
		fileMenuBar.setAnimationEnabled(true);		
		fileMenuBar.addItem("Load Scenario", new Command() {
			public void execute() {
				
			}
		});
		fileMenuBar.addItem("Recently Used", new Command() {
			public void execute() {
				
			}
		});
		fileMenuBar.addSeparator();
		fileMenuBar.addItem("Adjust Scenario", new Command() {
			public void execute() {
				
			}
		});
		fileMenuBar.addItem("Save", new Command() {
			public void execute() {
				
			}
		});
		fileMenuBar.addItem("Save As", new Command() {
			public void execute() {
				
			}
		});
		fileMenuBar.addSeparator();
		fileMenuBar.addItem("Print", new Command() {
			public void execute() {
				
			}
		});
		fileMenuBar.addItem("Exit", new Command() {
			public void execute() {
				
			}
		});

		//edit menu
		
		editMenuBar = new MenuBar(true);
		editMenuBar.setAnimationEnabled(true);
		editMenuBar.addItem("Undo", new Command() {
			public void execute() {
				
			}
		});
		editMenuBar.addItem("Redo", new Command() {
			public void execute() {
				
			}
		});
		editMenuBar.addSeparator();
		editMenuBar.addItem("Define Job Set", new Command() {
			public void execute() {
				
			}
		});
		editMenuBar.addItem("Edit Jobs", new Command() {
			public void execute() {
				
			}
		});
		editMenuBar.addItem("Change Database", new Command() {
			public void execute() {
				
			}
		});
		editMenuBar.addItem("Simulation Settings", new Command() {
			public void execute() {
				
			}
		});
		editMenuBar.addItem("Start/Stop Simulation", new Command() {
			public void execute() {
				
			}
		});
		editMenuBar.addSeparator();
		editMenuBar.addItem("Save As Image", new Command() {
			public void execute() {
				
			}
		});
		editMenuBar.addItem("PlugIn", new Command() {
			public void execute() {
				
			}
		});

		//view menu
		
		viewMenuBar = new MenuBar(true);
		viewMenuBar.setAnimationEnabled(true);
		viewMenuBar.addItem("Brightness", new Command() {
			public void execute() {
				
			}
		});
		viewMenuBar.addSeparator();
		viewMenuBar.addItem("Zoom In", new Command() {
			public void execute() {
				
			}
		});
		viewMenuBar.addItem("Zoom Out", new Command() {
			public void execute() {
				
			}
		});
		viewMenuBar.addSeparator();
		viewMenuBar.addItem("Fullscreen", new Command() {
			public void execute() {
				
			}
		});
		
		//menu bar
		
		menuBar = new MenuBar();
		menuBar.addItem("File", fileMenuBar);
		menuBar.addSeparator();
		menuBar.addItem("Edit", editMenuBar);
		menuBar.addSeparator();
		menuBar.addItem("View", viewMenuBar);
		menuBar.addSeparator();
		menuBar.addItem("Help", new Command() {
			public void execute() {
				
			}
		});
		
		//--- left panel ---

		//table
		
	    jobTable.setPageSize(5);
	    TextColumn<Job> jobNumberColumn = new TextColumn<Job>() {
	    	@Override
	        public String getValue(Job object) {
	    		return "" + object.jobNumber;
	        }
	    };
	    jobTable.addColumn(jobNumberColumn, "Job Number");	    
	    TextColumn<Job> jobColumn = new TextColumn<Job>() {
	    	@Override
	        public String getValue(Job object) {
	    		return object.job;
	        }
	    };
	    jobTable.addColumn(jobColumn, "Job");
	    vpLeftFrame.add(jobTable);

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
	    provider.addDataDisplay(jobTable);
	    provider.updateRowCount(JOBS.size(), true);
	    
	    SimplePager pager = new SimplePager();
	    pager.setDisplay(jobTable);

	    VerticalPanel vp = new VerticalPanel();
	    vp.add(jobTable);
	    vp.add(pager);
	    vpLeftFrame.add(vp);
	    
	    //togglebutton for switching between hybrid and virtual mode
	    
		tbVirtualHybridSwitch = new ToggleButton("Virtual / Hybrid");
		tbVirtualHybridSwitch.setText("Virtual / Hybrid");
		vpLeftFrame.add(tbVirtualHybridSwitch);
		
		//button for changing pathfinding strategy
		
		btStrategies = new Button("Strategies");
		btStrategies.setText("Strategies");
		vpLeftFrame.add(btStrategies);
		
		hpSubFrame.add(vpLeftFrame);
		
		canvas = Canvas.createIfSupported();
		hpSubFrame.add(canvas);
		
		vpMainFrame.add(menuBar);
		vpMainFrame.add(hpSubFrame);
		initWidget(vpMainFrame);
	}
	
	private static class Job {
	    private final int jobNumber;
	    private final String job;

	    public Job(int jobNumber, String job) {
	    	this.jobNumber = jobNumber;
	      	this.job = job;
	    }
	}
}