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
 * Dialogbox is used to allow the user to choose a name for the szenario
 * 
 * @author Raschid
 */
public class DialogBoxSaveAs extends DialogBox {

	protected static Object getPresenter;
	private MainFramePresenter presenter;
	private TextBox textbox;

	// The Dialog needs the reference to the Presenter in order to load it
	public DialogBoxSaveAs(MainFramePresenter presenter) {

		this.presenter = presenter;
		this.textbox=new TextBox();
		this.textbox.setText(this.presenter.getActualSzenario().getTitle());

		// Set the dialog box's caption.
		setText("Choose a name for the Szenario");

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
				if(DialogBoxSaveAs.this.textbox.getText().isEmpty()){
			         Window.alert("Please Select a name for the scenario");
			         return;
		        }else {
		        	//If scenario is not empty then you have to set the text for the scenario variable
		        	DialogBoxSaveAs.this.presenter.getActualSzenario().setTitle(DialogBoxSaveAs.this.textbox.getText());
		        	
		        }
				
				//If the Button was clicked it must been checked if a scenario with the given title exists
				
				// Check if Szenario already exists
				((SimulationServiceAsync) ((MainFramePresenter) DialogBoxSaveAs.this.presenter).getService()).checkIfTitleExists(
						DialogBoxSaveAs.this.getPresenter().getActualSzenario().getTitle(), new AsyncCallback<Boolean>() {

							public void onFailure(Throwable caught) {
								Window.alert("Communication Problem");

							}

							public void onSuccess(Boolean result) {
								// If Szenario exists open a popup, where the user can
								// be asked if he wants to overwritte or not
								if(result){
									//Show the popup so the user can select if he wants to overwrite or not
									DialogBoxOverwrite dialog = new DialogBoxOverwrite(
											DialogBoxSaveAs.this.getPresenter());
									dialog.show();
								}else {
									//Szenario is inserted into Database
									DialogBoxSaveAs.this.getPresenter().sendSzenarioToServer(DialogBoxSaveAs.this.getPresenter().getActualSzenario());
								}

							}

						});
				
				DialogBoxSaveAs.this.hide();
			}
		});

		hPanel.add(save);

		Button exit = new Button("exit");
		exit.addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				DialogBoxSaveAs.this.hide();
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
