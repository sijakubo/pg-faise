package uni.oldenburg.client.view;

import uni.oldenburg.client.presenter.MainFramePresenter;
import uni.oldenburg.client.service.SimulationServiceAsync;

import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.user.client.Window;
import com.google.gwt.user.client.rpc.AsyncCallback;
import com.google.gwt.user.client.ui.Button;
import com.google.gwt.user.client.ui.DialogBox;
import com.google.gwt.user.client.ui.HasHorizontalAlignment;
import com.google.gwt.user.client.ui.HorizontalPanel;
import com.google.gwt.user.client.ui.TextBox;
import com.google.gwt.user.client.ui.VerticalPanel;

/**
 * Dialogbox is used to allow the user to choose a name for the Joblist
 * 
 * @author Raschid
 */
public class DialogBoxSaveAsJoblist extends DialogBox {

	protected static Object getPresenter;
	private MainFramePresenter presenter;
	private TextBox textbox;

	// The Dialog needs the reference to the Presenter in order to load it
	public DialogBoxSaveAsJoblist(MainFramePresenter presenter) {

		this.presenter = presenter;
		this.textbox=new TextBox();
		this.textbox.setText(this.presenter.getActualJoblist().getName());

		// Set the dialog box's caption.
		setText("Choose a name for the Joblist");

		// Enable animation.
		setAnimationEnabled(true);

		// Enabl glass background.
		setGlassEnabled(true);
		
		
		VerticalPanel vpPanel=new VerticalPanel();

		HorizontalPanel hPanel = new HorizontalPanel();
		hPanel.setHorizontalAlignment(HasHorizontalAlignment.ALIGN_RIGHT);
		
		
		

		// DialogBox is a SimplePanel, so you have to set its widget property to
		// whatever you want its contents to be.

		
		Button save = new Button("save");
		save.addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				//Check if there is content within the textbox
				if(DialogBoxSaveAsJoblist.this.textbox.getText().isEmpty()){
			         Window.alert("Please Select a name for the Joblist");
			         return;
		        }else {
		        	//If Joblist is not empty then you have to set the text for the joblist variable
		        	DialogBoxSaveAsJoblist.this.presenter.getActualJoblist().setName(DialogBoxSaveAsJoblist.this.textbox.getText());
		        	
		        }
				
				//If the Button was clicked it must been checked if a Joblist with the given title exists
				
				// Check if Joblist already exists
				((SimulationServiceAsync) ((MainFramePresenter) DialogBoxSaveAsJoblist.this.presenter).getService()).checkIfJobListExists(
						DialogBoxSaveAsJoblist.this.getPresenter().getActualJoblist().getName(), new AsyncCallback<Boolean>() {

							public void onFailure(Throwable caught) {
								Window.alert("Communication Problem");

							}

							public void onSuccess(Boolean result) {
								// If Joblist exists open a popup, where the user can
								// be asked if he wants to overwritte or not
								if(result){
									//Show the popup so the user can select if he wants to overwrite or not
									DialogBoxOverwriteJoblist dialog = new DialogBoxOverwriteJoblist(
											DialogBoxSaveAsJoblist.this.getPresenter());
									dialog.show();
								}else {
									//Joblist is inserted into Database
									DialogBoxSaveAsJoblist.this.getPresenter().sendJoblistToServer(DialogBoxSaveAsJoblist.this.getPresenter().getActualJoblist());
								}

							}

						});
				
				DialogBoxSaveAsJoblist.this.hide();
			}
		});

		hPanel.add(save);

		Button exit = new Button("exit");
		exit.addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				DialogBoxSaveAsJoblist.this.hide();
			}
		});

		hPanel.add(exit);
		vpPanel.add(textbox);
		vpPanel.add(hPanel);
		

		this.setWidget(vpPanel);
		this.center();
	}

	public MainFramePresenter getPresenter() {
		return presenter;
	}

	public TextBox getTextbox() {
		return textbox;
	}

}
