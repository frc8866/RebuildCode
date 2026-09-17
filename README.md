OffSeason Code for 8866 for 2026 Rebuilt season

Procedures
1. You need WPILib 2026 and National Instruments FRC tools downloaded on a laptop.
2. Make sure USB drive for logging is plugged into roboRIO.
3. To start match, powercycle with intake out and shooter down, then push intake in while robot is on. Once the radio boots up (flashing green lights turn solid green), connect to network. It takes a minute for the network to appear.
4. To open photonvision, try photonvision.localhost:5800 then try 10.88.66.13:5800. turn auto exposure off and back on to fix cams or change gain if you need to. Make sure 
5. Open AdvantageKit and go to App -> Show Preferences. Set Robot Address as 10.88.66.2. Click the 3d Field tab. Go to File -> Connect to Robot -> Click NetworkTables 4. Drag NT:/AdvantageKit/RealOutputs/RobotPose from the table on the left to the Poses area under the field. Make sure Team Number is set as 8866 in DriverStation setup tab.
6. Make sure your controller is plugged in, rescan it in the USB Devices tab in Driver Station if you need to.
7. After drive practice is done, download logs from AdvantageScope (CTRL + D) or from the USB drive to your computer.
