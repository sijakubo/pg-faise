package uni.oldenburg.client.view;

import java.util.ArrayList;

import uni.oldenburg.client.presenter.MainFramePresenter;

import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.user.client.ui.Button;
import com.google.gwt.user.client.ui.DialogBox;
import com.google.gwt.user.client.ui.HasHorizontalAlignment;
import com.google.gwt.user.client.ui.HorizontalPanel;

//Dialogbox for selecting and loading of a Scenario
public class DialogBoxScenarioSelection extends DialogBox {

	private ArrayList<String> titles;
	
	private MainFramePresenter presenter;
	

	
	
	//The Dialog needs the reference to the Presenter in order to load it
	public DialogBoxScenarioSelection(ArrayList<String> titles,MainFramePresenter presenter) {
		
		this.titles=titles;
		this.presenter=presenter;
		
		

		// Set the dialog box's caption.
		setText("Choose a Scenario");

		// Enable animation.
		setAnimationEnabled(true);

		// Enable glass background.
		setGlassEnabled(true);
		
		
		HorizontalPanel buttonPane = new HorizontalPanel();
        buttonPane.setHorizontalAlignment(HasHorizontalAlignment.ALIGN_RIGHT);

        
		
		
		
		

		// DialogBox is a SimplePanel, so you have to set its widget property to
		// whatever you want its contents to be.
		Button exit = new Button("Exit");
		exit.addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				DialogBoxScenarioSelection.this.hide();
			}
		});
		
		buttonPane.add(exit);
        
		
		
		
		//Create Button for each Scenario
		for(int i=0;i<titles.size();i++){
			Button button = new Button(titles.get(i));
			button.addClickHandler(new ClickHandlerWithButtonReference(button,this));
				
			
			buttonPane.add(button);
			
		}
		
		
		this.setWidget(buttonPane);
		
		

	}
	
	
	






	public MainFramePresenter getPresenter() {
		return presenter;
	}


	
	
	
	
	

	
}
