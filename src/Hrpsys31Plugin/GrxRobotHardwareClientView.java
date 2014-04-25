package com.generalrobotix.ui.view;

import java.util.ArrayList;
import java.util.Date;

import jp.go.aist.hrp.simulator.DynamicsSimulator;
import jp.go.aist.hrp.simulator.DynamicsSimulatorFactory;
import jp.go.aist.hrp.simulator.DynamicsSimulatorFactoryHelper;
import jp.go.aist.hrp.simulator.SensorState;
import jp.go.aist.hrp.simulator.WorldStateHolder;
import jp.go.aist.hrp.simulator.DynamicsSimulatorPackage.IntegrateMethod;
import jp.go.aist.hrp.simulator.DynamicsSimulatorPackage.LinkDataType;
import jp.go.aist.hrp.simulator.DynamicsSimulatorPackage.SensorOption;

import org.eclipse.jface.dialogs.MessageDialog;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.KeyAdapter;
import org.eclipse.swt.events.KeyEvent;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Text;

import OpenHRP.RobotHardwareService;
import OpenHRP.RobotHardwareServiceHelper;
import OpenHRP.StateHolderService;
import OpenHRP.StateHolderServiceHelper;
import OpenHRP.StateHolderServicePackage.CommandHolder;
import OpenHRP.RobotHardwareServicePackage.RobotStateHolder;
import OpenHRP.RobotHardwareServicePackage.SwitchStatus;
import RTC.ComponentProfile;
import RTC.ConnectorProfile;
import RTC.ConnectorProfileHolder;
import RTC.ExecutionContext;
import RTC.LifeCycleState;
import RTC.PortInterfaceProfile;
import RTC.PortProfile;
import RTC.PortService;
import RTC.RTObject;
import RTC.RTObjectHelper;

import com.generalrobotix.ui.GrxBaseItem;
import com.generalrobotix.ui.GrxBaseView;
import com.generalrobotix.ui.GrxBaseViewPart;
import com.generalrobotix.ui.GrxPluginManager;
import com.generalrobotix.ui.grxui.Activator;
import com.generalrobotix.ui.item.GrxModelItem;
import com.generalrobotix.ui.item.GrxWorldStateItem;
import com.generalrobotix.ui.item.GrxWorldStateItem.WorldStateEx;
import com.generalrobotix.ui.util.GrxCorbaUtil;
import com.generalrobotix.ui.util.GrxDebugUtil;
import com.generalrobotix.ui.util.GrxXmlUtil;

@SuppressWarnings("serial")
/**
 * @brief RobotHardware RTC client view
 */
public class GrxRobotHardwareClientView extends GrxBaseView {
	public static final String TITLE = "RobotHardware RTC Client";
	private static final int NOT_CONNECTED = 0;
	private static final int CONNECTING    = 1;
	private static final int CONNECTED     = 2;

	private GrxJythonPromptView jythonView_;

	private StateHolderService sholder_;
	private RobotHardwareService hwCtrl_;

	private DynamicsSimulator dynamics_;
	private GrxWorldStateItem currentItem_;
	private GrxModelItem currentModel_;

	private WorldStateHolder worldStateH_  = new WorldStateHolder();
	private RobotStateHolder robotStateH_  = new RobotStateHolder();

	private String robotType_ = "-----";
	private String robotHost_    = "localhost";
	private int    robotPort_    = 2809;
	private int    interval_  = 200;
	private String setupFile_ = "-----";
	private String RobotHardwareRTC_ = "RobotHardware0";
	private String StateHolderRTC_ = "StateHolder0";

	private Date initialDate_;
	private Date prevDate_;
	private int  state_       = NOT_CONNECTED;
	private boolean actualFlag = false;
	private Image startMonitorIcon_ = Activator.getDefault().getImage("sim_start.png");
	private Image stopMonitorIcon_  = Activator.getDefault().getImage("sim_stop.png");
	private Image servoOnIcon_  = Activator.getDefault().getImage("robot_servo_start.png");
	private Image servoOffIcon_ = Activator.getDefault().getImage("robot_servo_stop.png");
	private Image lampOnIcon_  = Activator.getDefault().getImage("lamp_on.png");
	private Image lampOffIcon_ = Activator.getDefault().getImage("lamp_off.png");
	private Button btnConnect_;
	private Button btnServo_;
	private Button btnSetup_;
	private Label  lblLamp_;
	private Label  lblStatus_;
	private Text   fldStatus_;
	private Composite propertyPanel_;
	private boolean isMonitorRunning_;

	private ArrayList<SetPropertyPanel> propList_ = new ArrayList<SetPropertyPanel>();

	/**
	 * @brief constructor
	 * @param name
	 * @param manager
	 * @param vp
	 * @param parent
	 */
	public GrxRobotHardwareClientView(String name, GrxPluginManager manager, GrxBaseViewPart vp, Composite parent) {
		super(name, manager, vp, parent);

		composite_.setLayout(new GridLayout(1,false));
		Composite layout1 = new Composite(composite_, SWT.NONE);
		layout1.setLayoutData(new GridData(GridData.HORIZONTAL_ALIGN_CENTER));
		GridLayout gridLayout = new GridLayout(6, false);
		gridLayout.marginWidth = 5;
		gridLayout.horizontalSpacing = 5;
		layout1.setLayout(gridLayout);
		// servo on/off button
		btnServo_ = new Button(layout1, SWT.TOGGLE);
		btnServo_.setImage(servoOnIcon_);
		GridData btnGridData = new GridData();
		btnGridData.widthHint = 32;
		btnGridData.heightHint = 32;
		btnServo_.setLayoutData(btnGridData);
		// status label
		lblStatus_ = new Label(layout1, SWT.NONE);
		lblStatus_.setText("Status");
		// lamp label
		lblLamp_ = new Label(layout1, SWT.NONE);
		lblLamp_.setImage(lampOffIcon_);
		// status filed
		fldStatus_ = new Text(layout1, SWT.SINGLE | SWT.BORDER); 
		fldStatus_.setText("Not Connected.");
		GridData textGridData = new GridData();
		textGridData.widthHint = 100;
		fldStatus_.setLayoutData(textGridData);
		// connect button
		btnConnect_ = new Button(layout1, SWT.TOGGLE);
		btnConnect_.setLayoutData(btnGridData);
		btnConnect_.setImage(startMonitorIcon_);
		// setup button
		btnSetup_ = new Button(layout1, SWT.NONE);
		btnSetup_.setText("ROBOT SETUP");

		propertyPanel_ = new Composite(composite_,SWT.NONE);
		propertyPanel_.setLayoutData(new GridData(GridData.HORIZONTAL_ALIGN_CENTER));
		GridLayout gridLayout1 = new GridLayout(1, false);
		gridLayout.marginWidth = 5;
		gridLayout.horizontalSpacing = 5;
		propertyPanel_.setLayout(gridLayout1);
		propList_.add(new SetPropertyPanel(propertyPanel_, SWT.NONE, "Robot Host",  "robotHost", true, robotHost_));
		propList_.add(new SetPropertyPanel(propertyPanel_, SWT.NONE, "Robot Port",  "robotPort", true, new Integer(robotPort_).toString()));
		propList_.add(new SetPropertyPanel(propertyPanel_, SWT.NONE, "Robot Name",  "ROBOT",     true, robotType_));
		propList_.add(new SetPropertyPanel(propertyPanel_, SWT.NONE, "Interval[ms]","interval",  true, new Integer(interval_).toString()));
		propList_.add(new SetPropertyPanel(propertyPanel_, SWT.NONE, "Setup File",  "setupFile", true, setupFile_));
		propList_.add(new SetPropertyPanel(propertyPanel_, SWT.NONE, "RobotHardwareService",  "RobotHardwareServiceRTC", true, RobotHardwareRTC_));
		propList_.add(new SetPropertyPanel(propertyPanel_, SWT.NONE, "StateHolderService",  "StateHolderRTC", true, StateHolderRTC_));

		btnConnect_.addSelectionListener(new SelectionListener() {
			public void widgetDefaultSelected(SelectionEvent e) {}
			public void widgetSelected(SelectionEvent e) {
				if (btnConnect_.getSelection()) {
					startMonitor(true);
				} else {
					boolean ans = MessageDialog.openConfirm(getParent().getShell(),
							"Do you really want to stop monitoring?",
					"Stopping Robot State Monitor");

					if (ans){
						stopMonitor();
					}else{
						btnConnect_.setSelection(true);
					}
				}
			}
		});

		btnSetup_.setEnabled(false);
		btnSetup_.addSelectionListener(new SelectionListener() {
			public void widgetDefaultSelected(SelectionEvent e) {}
			public void widgetSelected(SelectionEvent e) {
				setupFile_ = getProperty("setupFile");
				getJythonView().execFile(GrxXmlUtil.expandEnvVal(setupFile_));
			}
		});

		btnServo_.setEnabled(false);
		btnServo_.addSelectionListener(new SelectionListener() {
			public void widgetDefaultSelected(SelectionEvent e) {}
			public void widgetSelected(SelectionEvent e) {
			    if (btnServo_.getSelection()){
				servoOn();
			    }else{
					servoOff();
			    }
			}
		});

		currentItem_ = manager_.<GrxWorldStateItem>getSelectedItem(GrxWorldStateItem.class, null);
		if(currentItem_!=null){
			currentItem_.addObserver(this);
		}
		manager_.registerItemChangeListener(this, GrxWorldStateItem.class);
	}

	public void registerItemChange(GrxBaseItem item, int event){
		if(item instanceof GrxWorldStateItem){
			GrxWorldStateItem worldStateItem = (GrxWorldStateItem) item;
	    	switch(event){
	    	case GrxPluginManager.SELECTED_ITEM:
	    		if(currentItem_!=worldStateItem){
	    			currentItem_ = worldStateItem;
	    			currentItem_.addObserver(this);
	    		}
	    		break;
	    	case GrxPluginManager.REMOVE_ITEM:
	    	case GrxPluginManager.NOTSELECTED_ITEM:
	    		if(currentItem_==worldStateItem){
    				currentItem_.deleteObserver(this);
	    			currentItem_ = null;
	    		}
	    		break;
	    	default:
	    		break;
	    	}
		}
	}
	
	private class SetPropertyPanel extends Composite {
		private String propName;	
		private boolean isLocal = true;
		private String defaultVal;

		private Label    label;
		private Text  fld;
		private Button     set;
		private Button  resume;


		public SetPropertyPanel(Composite parent, int style, String _title, String _propName, boolean _isLocal, String _defaultVal) {
			super(parent, style);
			GridLayout gridLayout = new GridLayout(4, false);
			gridLayout.marginWidth = 5;
			gridLayout.horizontalSpacing = 5;
			this.setLayout(gridLayout);
			GridData labelGridData = new GridData();
			labelGridData.widthHint = 150;
			label = new Label(this, SWT.NONE);
			label.setText(_title);
			label.setLayoutData(labelGridData);
			fld = new Text(this, SWT.NONE);
			GridData textGridData = new GridData();
			textGridData.widthHint = 120;
			fld.setLayoutData(textGridData);
			set = new Button(this, SWT.NONE);
			set.setText("Set");
			resume = new Button(this, SWT.NONE);
			resume.setText("Resume");
			propName = _propName;
			isLocal = _isLocal;
			defaultVal = _defaultVal;

			fld.addKeyListener(new KeyAdapter() {
				public void keyReleased(KeyEvent e){
					if (e.keyCode == SWT.CR) {
						set(); 
					} else {
						boolean hasChanged = !fld.getText().equals(getValue());
						set.setEnabled(hasChanged);
						resume.setEnabled(hasChanged);
					}
				}
			});

			set.setEnabled(false);
			set.addSelectionListener(new SelectionListener() {
				public void widgetDefaultSelected(SelectionEvent e) {
				}
				public void widgetSelected(SelectionEvent e) {
					set();
				}
			});

			resume.setEnabled(false);
			resume.addSelectionListener(new SelectionListener() {
				public void widgetDefaultSelected(SelectionEvent e) {
				}
				public void widgetSelected(SelectionEvent e) {
					resume();
				}
			});
		}

		public void setEditable(boolean isEditable) {
			fld.setEditable(isEditable);
			resume.setEnabled(isEditable);
			set.setEnabled(isEditable);
		}

		public String getValue() {
			String str;
			if (isLocal)
				str = getProperty(propName);
			else
				str = manager_.getProjectProperty(propName);
			if (str == null)
				str = defaultVal;
			return str;
		}

		public void set() {
			String value = fld.getText();
			if (isLocal){
				setProperty(propName, value);
			}else{
				manager_.setProjectProperty(propName, value);
				System.out.println("("+propName+","+value+")"); 
			}
			set.setEnabled(false);
			resume.setEnabled(false);
		}

		public void resume() {
			fld.setText(getValue());
			set.setEnabled(false);
			resume.setEnabled(false);
		}
	}

	private void startMonitor(final boolean isInteractive) {
		if (isMonitorRunning_) return;
		
		isMonitorRunning_ = true;
		btnConnect_.setImage(stopMonitorIcon_);
		btnConnect_.setToolTipText("Stop Robot State Monitor");
		btnConnect_.setSelection(true);

		setConnectionState(CONNECTING);

		//GrxGuiUtil.setEnableRecursive(false, propertyPanel_, null);

		Runnable runnable = new Runnable() {
			public void run(){
				Display display = composite_.getDisplay();
				if (state_ == CONNECTING){
					tryConnection(isInteractive);
					if (!display.isDisposed())
						display.timerExec(1000, this);
				}else if (state_ == CONNECTED){
					try{
					updateRobotState();
					}catch(Exception ex){
						ex.printStackTrace();
					}
					interval_  = getInt("interval", 200);
					if (!display.isDisposed())
						display.timerExec(interval_, this);
				}else{
				}
			}
		};
		Display display = composite_.getDisplay();
		if (!display.isDisposed()){
			display.timerExec(1, runnable);
		}


	}

    private void tryConnection(boolean isInteractive){
	System.out.println("tryConnection()");
	if (currentItem_ == null) return;
		try {
			for (int i=0; i<propList_.size(); i++)
				propList_.get(i).resume();

			// set property for connection
			robotHost_ = getStr("robotHost");
			if (robotHost_ != null){
				manager_.setProjectProperty("nsHost", robotHost_);
			}
			try {
				robotPort_ = Integer.parseInt(manager_.getProjectProperty("robotPort"));
			} catch (Exception e) {
				robotPort_ = 2809; // if not set try the default port number
			}

			robotType_ = getStr("ROBOT");

			currentModel_ = (GrxModelItem) manager_.getItem(GrxModelItem.class, robotType_);
			if (currentModel_ == null){
			    System.out.println("Can't find robot model("+robotType_+")"); 
				return;
			}

			//actualFlag = isTrue("showActualState", false);

			// initialize logger
			currentItem_.clearLog();
			currentItem_.registerCharacter(robotType_, currentModel_.getBodyInfo());

			// get RobotHardwareService
			System.out.println("RobotHardwareRTC_ = "+RobotHardwareRTC_);
			RTObject rtc = findRTC(RobotHardwareRTC_);
			ExecutionContext ec = rtc.get_owned_contexts()[0];
			if (ec.get_component_state(rtc) != LifeCycleState.ACTIVE_STATE){
				ec.activate_component(rtc);
			}
			hwCtrl_ = RobotHardwareServiceHelper.narrow(findService(rtc, "service0"));

			// initialize servo On button
			if (isAnyServoOn()) {
				btnServo_.setSelection(true);
				btnServo_.setImage(servoOffIcon_);
				btnServo_.setToolTipText("Servo Off");
			} else {
				btnServo_.setSelection(false);
				btnServo_.setImage(servoOnIcon_);
				btnServo_.setToolTipText("Servo On");
			}
			prevDate_ = new Date();

			initDynamicsSimulator(false);

			setConnectionState(CONNECTED);

		} catch (Exception e) {
			//GrxDebugUtil.printErr("", e);

			if (isInteractive && currentModel_ == null) {
				MessageDialog.openInformation(getParent().getShell(),
						"Load Model(" + robotType_ + ") first.",
				"Start Robot State Monitor");
				stopMonitor();
			} else {
				setConnectionState(CONNECTING);
			}
		}
    }

	private void stopMonitor() {
	    btnConnect_.setImage(startMonitorIcon_);
		btnConnect_.setToolTipText("Start Robot State Monitor");
		btnConnect_.setSelection(false);

		setConnectionState(NOT_CONNECTED);
		isMonitorRunning_ = false;
	}

	private void setConnectionState(int state) {
		switch (state) {
		case NOT_CONNECTED:
			btnSetup_.setEnabled(false);
			btnServo_.setEnabled(false);
			fldStatus_.setText("Not Connected.");
			lblLamp_.setImage(lampOffIcon_);
			//GrxGuiUtil.setEnableRecursive(true, propertyPanel_, null);
			for (int i=0; i<propList_.size(); i++) {
				propList_.get(i).set.setEnabled(false);
				propList_.get(i).resume.setEnabled(false);
			}

			break;

		case CONNECTING:
			btnSetup_.setEnabled(false);
			btnServo_.setEnabled(false);
			fldStatus_.setText("Connecting ...");
			//GrxGuiUtil.setEnableRecursive(false, propertyPanel_, null);
			break;

		case CONNECTED:
			btnSetup_.setEnabled(true);
			btnServo_.setEnabled(true);
			fldStatus_.setText("Connected.");
			lblLamp_.setImage(lampOnIcon_);
			break;
		}
		state_ = state;
	}

	public void restoreProperties() {
		super.restoreProperties();
		robotHost_ = getStr("robotHost");
		if (robotHost_ != null){
			manager_.setProjectProperty("nsHost", robotHost_);

		}
		startMonitor(false);
	}

	public RTObject findRTC(String name){
		org.omg.CORBA.Object obj = GrxCorbaUtil.getReference(name, "rtc", robotHost_, robotPort_);
		return RTObjectHelper.narrow(obj);
	}

	public org.omg.CORBA.Object findService(RTObject rtc, String service){
		ComponentProfile prof = rtc.get_component_profile();
		PortProfile[] port_prof = prof.port_profiles;
		PortService port = null;
		for (int i=0; i<port_prof.length; i++){
			PortProfile pp = port_prof[i];
			PortInterfaceProfile[] ifs = pp.interfaces;
			for (int j=0; j<ifs.length; j++){
				PortInterfaceProfile aif = ifs[j];
				if (aif.instance_name.equals(service)){
					port = pp.port_ref;
				}
			}
		}
		ConnectorProfile con_prof = new ConnectorProfile();
		con_prof.name = "noname";
		con_prof.connector_id = "";
		con_prof.ports = new PortService[1];
		con_prof.ports[0] = port;
		con_prof.properties = new _SDOPackage.NameValue[0];
		ConnectorProfileHolder con_prof_holder = new ConnectorProfileHolder();
		con_prof_holder.value = con_prof;
		port.connect(con_prof_holder);
		String ior = con_prof_holder.value.properties[0].value.extract_string();
		port.disconnect(con_prof_holder.value.connector_id);
		return GrxCorbaUtil.getORB().string_to_object(ior);
	} 

	// get the data here and stuff it into currentItem_
    private void updateRobotState(){
		//if (currentItem_ == null || !btnConnect_.isSelected()) {
		if (currentItem_ == null) {
			stopMonitor();
			return;
		}
		
		Date now = new Date();
		long time = now.getTime();
		if (prevDate_ != null) {
			time -= prevDate_.getTime();
		}
		if (state_ == CONNECTING) {
			if (time > 1000) {
				prevDate_ = now;
				if (lblLamp_.getImage() == lampOffIcon_)
					lblLamp_.setImage(lampOnIcon_);
				else
					lblLamp_.setImage(lampOffIcon_);
				startMonitor(false);
			}
			return;
		}

		if (state_ == CONNECTED && time > interval_) {
			prevDate_ = now;
			try {
				if (hwCtrl_ != null){
					hwCtrl_.getStatus(robotStateH_);
				}

				if(actualFlag) {
					dynamics_.setCharacterAllLinkData(robotType_, LinkDataType.JOINT_VALUE, robotStateH_.value.angle);
				} else {
					//dynamics_.setCharacterAllLinkData(robotType_, LinkDataType.JOINT_VALUE, robotStateH_.value.command);
				}
				
				try{
					if (sholder_ == null){
						StateHolderRTC_ = getStr("StateHolderRTC", StateHolderRTC_);
						RTObject rtc = findRTC(StateHolderRTC_);
						sholder_ = StateHolderServiceHelper.narrow(findService(rtc, "service0"));
					}
					CommandHolder com = new CommandHolder();
					sholder_.getCommand(com);
					dynamics_.setCharacterLinkData(robotType_, currentModel_.rootLink().getName(), LinkDataType.ABS_TRANSFORM, com.value.baseTransform);
					dynamics_.setCharacterAllLinkData(robotType_, LinkDataType.JOINT_VALUE, robotStateH_.value.command);
				}catch(Exception e){
					GrxDebugUtil.printErr("failed to connect StateHolderService");
					sholder_ = null;
				}

				dynamics_.calcWorldForwardKinematics();
				dynamics_.getWorldState(worldStateH_);

			} catch (Exception e) {
				GrxDebugUtil.printErr("iobclient got exception:", e);
				setConnectionState(CONNECTING);
				return;
			}

			SensorState ss = new SensorState();
			ss.q = robotStateH_.value.angle;
			ss.u = robotStateH_.value.torque;
			ss.force = robotStateH_.value.force;
			ss.accel = robotStateH_.value.accel;
			ss.rateGyro = robotStateH_.value.rateGyro;

			WorldStateEx wsx = new WorldStateEx(worldStateH_.value);
			if (currentItem_.getLogSize() == 0){
				initialDate_ = now;
			}
			wsx.time = (now.getTime() - initialDate_.getTime()) / 1000.0;
			wsx.collisions = null;
			wsx.setSensorState(robotType_, ss);
			wsx.setTargetState(robotType_, robotStateH_.value.command);
			int [] sstate = new int[robotStateH_.value.servoState.length];
			for (int i=0; i<sstate.length; i++){
			    sstate[i] = robotStateH_.value.servoState[i][0];
			}
			wsx.setServoState(robotType_, sstate);
			wsx.setPowerState(robotType_, robotStateH_.value.voltage, robotStateH_.value.current);
			currentItem_.addValue(wsx.time, wsx);
			currentItem_.setPosition(currentItem_.getLogSize()-1);
		}
	}

	private DynamicsSimulator initDynamicsSimulator(boolean update) {
		if (dynamics_ != null && !update) {
			return dynamics_;
		}
		org.omg.CORBA.Object obj = GrxCorbaUtil.getReference("DynamicsSimulatorFactory");
		DynamicsSimulatorFactory dynFactory = DynamicsSimulatorFactoryHelper.narrow(obj);
		if (dynamics_ != null)  {
			try {
				dynamics_.destroy();
			} catch(Exception e) {
				GrxDebugUtil.printErr("", e);
			}
		}
		try {
			dynamics_ = dynFactory.create();
			dynamics_.registerCharacter(robotType_, currentModel_.getBodyInfo());
			dynamics_.init(0.001, IntegrateMethod.EULER, SensorOption.DISABLE_SENSOR);
		} catch (Exception e) {
			dynamics_ = null;
			e.printStackTrace();
		}

		return dynamics_;
	}

	private GrxJythonPromptView getJythonView() {
		if (jythonView_ == null) {
			jythonView_= (GrxJythonPromptView)manager_.getView(GrxJythonPromptView.class,true);
		}

		return jythonView_;
	}

	private void servoOn() {
		if (isAnyServoOn()) {
			return;
		}
		if(hwCtrl_ != null) {
			hwCtrl_.power("all", SwitchStatus.SWITCH_ON);
			try{
				Thread.sleep(1000);
			}catch(Exception ex){
				ex.printStackTrace();
			}
		}

		boolean ans = MessageDialog.openConfirm(getParent().getShell(),
				"!! Robot Motion Warning (SERVO ON) !!\n" + 
				"Confirm RELAY turned ON.\n" + 
				"Then Push [OK] to servo ON.\n", 
		"Servo ON");
		if (ans) {
			try {
				// get StateHolderService
				StateHolderRTC_ = getStr("StateHolderRTC", StateHolderRTC_);
				RTObject rtc = findRTC(StateHolderRTC_);
				sholder_ = StateHolderServiceHelper.narrow(findService(rtc, "service0"));

				if (sholder_ != null) {
					sholder_.goActual();
					try{
						Thread.sleep(100);
					}catch(Exception ex){
						ex.printStackTrace();
					}
				}

				if(hwCtrl_ != null){
					hwCtrl_.servo("all", SwitchStatus.SWITCH_ON);
					try {
						Thread.sleep(5000);
					}catch(Exception ex){
						ex.printStackTrace();
					}
				}

				btnServo_.setImage(servoOffIcon_);
				btnServo_.setToolTipText("Servo Off");
			} catch (Exception e) {
				if(hwCtrl_ != null)
					hwCtrl_.power("all", SwitchStatus.SWITCH_OFF);
				GrxDebugUtil.printErr("got exception during servo on process:", e);
			}
		} else {
			if(hwCtrl_ != null)
				hwCtrl_.power("all", SwitchStatus.SWITCH_OFF);
			btnServo_.setSelection(false);
		}
	}

	private void servoOff() {
		boolean ans = MessageDialog.openConfirm(getParent().getShell(),
				"!! Robot Motion Warning (SERVO OFF) !!\n\n" + 
				"Push [OK] to servo OFF.\n", 
		"Servo OFF");
		if (ans) {
			try {
				if(hwCtrl_ != null) {
					hwCtrl_.servo("all", SwitchStatus.SWITCH_OFF);
					hwCtrl_.power("all", SwitchStatus.SWITCH_OFF);
				}
				btnServo_.setImage(servoOnIcon_);
				btnServo_.setToolTipText("Servo On");
			} catch (Exception e) {
				GrxDebugUtil.printErr("got exception during servo off process:", e);
			}
		} else {
		    btnServo_.setSelection(true);
		}
	}

	public boolean isAnyServoOn() {
		final int SERVO_STATE_MASK = 0x2;
		if(hwCtrl_ != null) {
			hwCtrl_.getStatus(robotStateH_);
			int[][] state = robotStateH_.value.servoState;
			for (int i=0; i<currentModel_.getDOF(); i++) {
				if ((state[i][0]&SERVO_STATE_MASK) != 0) {
					return true;
				}
			}
		}
		return false;
	}	
}