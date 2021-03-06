package uni.oldenburg.client.presenter;

import java.util.Arrays;
import java.util.List;

import uni.oldenburg.client.service.ServiceAsync;
import uni.oldenburg.shared.model.Job;

import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.event.dom.client.HasClickHandlers;
import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.cellview.client.CellTable;
import com.google.gwt.user.cellview.client.TextColumn;
import com.google.gwt.user.client.ui.Widget;
import com.google.gwt.view.client.AsyncDataProvider;
import com.google.gwt.view.client.HasData;

public class MainFramePresenter extends Presenter {
    private final IDisplay display;

	private static final List<Job> JOBS = Arrays.asList(
		      new Job(1, "XY"),
		      new Job(2, "zt"),
		      new Job(3, "Patata"),
		      new Job(4, "UO"));
		
	public interface IDisplay {
        CellTable<Job>   getJobTable();
        HasClickHandlers getStrategiesButton();
        HasClickHandlers getVirtualHybridButton();
    }
	
    public MainFramePresenter(ServiceAsync rpcService, HandlerManager eventBus, IDisplay view) {
    	super(rpcService, eventBus);
        this.display = view;
    }
    
	public Widget getDisplay() {
		return (Widget)display;
	}    

    private void addStrategiesButtonListener() {
        display.getStrategiesButton().addClickHandler(new ClickHandler() {
            public void onClick(ClickEvent event) {
                //TODO
            }
        });
    }

    private void addVirtualHybridButtonListener() {
        display.getVirtualHybridButton().addClickHandler(new ClickHandler() {
            public void onClick(ClickEvent event) {
                //TODO
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
	    display.getJobTable().addColumn(jobNumberColumn, "Job Number");	    
	    TextColumn<Job> jobColumn = new TextColumn<Job>() {
	    	@Override
	        public String getValue(Job object) {
	    		return object.getJob();
	        }
	    };
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

    public void bind() {
        this.addStrategiesButtonListener();
        this.addVirtualHybridButtonListener();
        this.setupJobTable();
    }
}
