/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

//#include "render/render.h"
#include "highway.h"

int main(int argc, char** argv)
{

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	// set camera position and angle
	viewer->initCameraParameters();
	float x_pos = 0;
	viewer->setCameraPosition ( x_pos-26, 0, 15.0, x_pos+25, 0, 0, 0, 0, 1);

	Highway highway(viewer);

	//initHighway(viewer);

	int frame_per_sec = 30;
	int sec_interval = 10;
	int frame_count = 0;
	int time_us = 0;

	double egoVelocity = 25;

	// output results for postprocess analysis
	highway.NIS_ouput.open("NIS.csv");
	highway.NIS_ouput<<"Time, Lidar NIS, Radar NIS \n";

	highway.groundTruth_data.open("groundTruth_data.csv");
	highway.groundTruth_data<<"Time, ground truth x, ground truth y, ground truth v, ground truth yaw \n";

	highway.estimated_data.open("estimated_data.csv");
	highway.estimated_data<<"Time, estimated x, estimated y, estimated v, estimated yaw \n";	

	highway.lidarMeasured_data.open("lidarMeasured_data.csv");
	highway.lidarMeasured_data<<"Time, lidar measured x, lidar measured y \n";

	highway.radarMeasured_data.open("radarMeasured_data.csv");
	highway.radarMeasured_data<<"Time, radar measured  rho, radar measured phi, radar measured rho_dot \n";
	
	

	while (frame_count < (frame_per_sec*sec_interval))
	{
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		//stepHighway(egoVelocity,time_us, frame_per_sec, viewer);
		highway.stepHighway(egoVelocity,time_us, frame_per_sec, viewer);

		// save the screen shot from the video 
		/*
		bool bSave = false;
		if (bSave) 
		{
			
			ostringstream imgNumber;
			imgNumber << setfill('0') << setw(3) << frame_count;
			string imgFullFilename =  "saved_" + imgNumber.str() + ".png";
			viewer->saveScreenshot(imgFullFilename);
		} */

		viewer->spinOnce(1000/frame_per_sec);
		frame_count++;
		time_us = 1000000*frame_count/frame_per_sec;
	}

	// close the file
	highway.NIS_ouput.close();
	highway.groundTruth_data.close();
	highway.estimated_data.close();
	highway.lidarMeasured_data.close();
	highway.radarMeasured_data.close();
}