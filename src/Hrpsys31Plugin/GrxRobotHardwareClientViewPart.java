package com.generalrobotix.ui.view;

import org.eclipse.swt.widgets.Composite;
import com.generalrobotix.ui.GrxBaseViewPart;

public class GrxRobotHardwareClientViewPart extends GrxBaseViewPart{

	public void createPartControl(Composite parent) {
		createView( GrxRobotHardwareClientView.class, "RobotHardware RTC Client", this, parent );
	}
}
