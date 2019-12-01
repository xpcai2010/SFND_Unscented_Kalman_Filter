/* \author Aaron Brown */
// Handle logic for creating traffic on highway and animating it

#include "render/render.h"
#include "sensors/lidar.h"
#include "tools.h"

#include <iostream>
#include <fstream>


class Highway
{
public:

	std::vector<Car> traffic;
	Car egoCar;
	Tools tools;
	bool pass = true;
	std::vector<double> rmseThreshold = {0.30,0.16,0.95,0.70};
	std::vector<double> rmseFailLog = {0.0,0.0,0.0,0.0};
	Lidar* lidar;
	
	// Parameters 
	// --------------------------------
	// Set which cars to track with UKF
	std::vector<bool> trackCars = {true,true,true};
	// Visualize sensor measurements
	bool visualize_lidar = true;
	bool visualize_radar = true;
	bool visualize_pcd = false;
	// Predict path in the future using UKF
	double projectedTime = 1;
	int projectedSteps = 5;
	// --------------------------------

	// output NIS values to file
	std::ofstream NIS_ouput;

	// output ground truth, ukf estimated, lidar and radar measured data to file
	std::ofstream groundTruth_data;
	std::ofstream estimated_data;
	std::ofstream lidarMeasured_data;
	std::ofstream radarMeasured_data;

	Highway(pcl::visualization::PCLVisualizer::Ptr& viewer)
	{

		tools = Tools();
	
		egoCar = Car(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), 0, 0, 2, "egoCar");
		
		Car car1(Vect3(-10, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), 5, 0, 2, "car1");
		
		std::vector<accuation> car1_instructions;
		accuation a = accuation(0.5*1e6, 0.5, 0.0);
		car1_instructions.push_back(a);
		a = accuation(2.2*1e6, 0.0, -0.2);
		car1_instructions.push_back(a);
		a = accuation(3.3*1e6, 0.0, 0.2);
		car1_instructions.push_back(a);
		a = accuation(4.4*1e6, -2.0, 0.0);
		car1_instructions.push_back(a);
	
		car1.setInstructions(car1_instructions);
		if( trackCars[0] )
		{
			UKF ukf1;
			car1.setUKF(ukf1);
		}
		traffic.push_back(car1);
		
		Car car2(Vect3(25, -4, 0), Vect3(4, 2, 2), Color(1, 0, 0), -6, 0, 2, "car2");
		std::vector<accuation> car2_instructions;
		a = accuation(4.0*1e6, 3.0, 0.0);
		car2_instructions.push_back(a);
		a = accuation(8.0*1e6, 0.0, 0.0);
		car2_instructions.push_back(a);
		car2.setInstructions(car2_instructions);
		if( trackCars[1] )
		{
			UKF ukf2;
			car2.setUKF(ukf2);
		}
		traffic.push_back(car2);
	
		Car car3(Vect3(-12, 0, 0), Vect3(4, 2, 2), Color(0.5, 0, 0.5), 1, 0, 2, "car3");
		std::vector<accuation> car3_instructions;
		a = accuation(0.5*1e6, 2.0, 1.0);
		car3_instructions.push_back(a);
		a = accuation(1.0*1e6, 2.5, 0.0);
		car3_instructions.push_back(a);
		a = accuation(3.2*1e6, 0.0, -1.0);
		car3_instructions.push_back(a);
		a = accuation(3.3*1e6, 2.0, 0.0);
		car3_instructions.push_back(a);
		a = accuation(4.5*1e6, 0.0, 0.0);
		car3_instructions.push_back(a);
		a = accuation(5.5*1e6, -2.0, 0.0);
		car3_instructions.push_back(a);
		a = accuation(7.5*1e6, 0.0, 0.0);
		car3_instructions.push_back(a);
		car3.setInstructions(car3_instructions);
		if( trackCars[2] )
		{
			UKF ukf3;
			car3.setUKF(ukf3);
		}
		traffic.push_back(car3);

		lidar = new Lidar(traffic,0);
	
		// render environment
		renderHighway(0,viewer);
		egoCar.render(viewer);
		car1.render(viewer);
		car2.render(viewer);
		car3.render(viewer);
	}
	
	void stepHighway(double egoVelocity, long long timestamp, int frame_per_sec, pcl::visualization::PCLVisualizer::Ptr& viewer)
	{

		if(visualize_pcd)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr trafficCloud = tools.loadPcd("../src/sensors/data/pcd/highway_"+std::to_string(timestamp)+".pcd");
			renderPointCloud(viewer, trafficCloud, "trafficCloud", Color((float)184/256,(float)223/256,(float)252/256));
		}
		

		// render highway environment with poles
		renderHighway(egoVelocity*timestamp/1e6, viewer);
		egoCar.render(viewer);
		
		for (int i = 0; i < traffic.size(); i++)
		{
			traffic[i].move((double)1/frame_per_sec, timestamp);
			if(!visualize_pcd)
				traffic[i].render(viewer);
			// Sense surrounding cars with lidar and radar
			if(trackCars[i])
			{
				VectorXd gt(4);
				gt << traffic[i].position.x, traffic[i].position.y, traffic[i].velocity*cos(traffic[i].angle), traffic[i].velocity*sin(traffic[i].angle);
				tools.ground_truth.push_back(gt);

				// ground truth data output
				groundTruth_data<<traffic[i].ukf.previous_timestamp_<<","<<traffic[i].position.x<<","<<traffic[i].position.y<<","<<traffic[i].velocity<<","<<traffic[i].angle<<"\n";
				
				lmarker lidar_meas_package = tools.lidarSense(traffic[i], viewer, timestamp, visualize_lidar);
				// lidar measurement data output
				lidarMeasured_data<<traffic[i].ukf.previous_timestamp_<<","<<lidar_meas_package.x<<","<<lidar_meas_package.y<<"\n";

				rmarker radar_meas_package = tools.radarSense(traffic[i], egoCar, viewer, timestamp, visualize_radar);
				// radar measurement data output
				radarMeasured_data<<traffic[i].ukf.previous_timestamp_<<","<<radar_meas_package.rho<<","<<radar_meas_package.phi<<","<<radar_meas_package.rho_dot<<"\n";

				tools.ukfResults(traffic[i],viewer, projectedTime, projectedSteps);
				VectorXd estimate(4);
				double v  = traffic[i].ukf.x_(2);
    			double yaw = traffic[i].ukf.x_(3);
    			double v1 = cos(yaw)*v;
    			double v2 = sin(yaw)*v;
				estimate << traffic[i].ukf.x_[0], traffic[i].ukf.x_[1], v1, v2;
				tools.estimations.push_back(estimate);

				// NIS evaluation output
				NIS_ouput << traffic[i].ukf.previous_timestamp_ << ","<< traffic[i].ukf.NIS_lidar_ <<","<< traffic[i].ukf.NIS_radar_<< "\n";
				// ukf estimated data output
				estimated_data<<traffic[i].ukf.previous_timestamp_<<","<<estimate[0]<<","<<estimate[1]<<","<<v<<","<<yaw<<"\n";

			}
		}
		viewer->addText("Accuracy - RMSE:", 30, 300, 20, 1, 1, 1, "rmse");
		VectorXd rmse = tools.CalculateRMSE(tools.estimations, tools.ground_truth);
		viewer->addText(" X: "+std::to_string(rmse[0]), 30, 275, 20, 1, 1, 1, "rmse_x");
		viewer->addText(" Y: "+std::to_string(rmse[1]), 30, 250, 20, 1, 1, 1, "rmse_y");
		viewer->addText("Vx: "	+std::to_string(rmse[2]), 30, 225, 20, 1, 1, 1, "rmse_vx");
		viewer->addText("Vy: "	+std::to_string(rmse[3]), 30, 200, 20, 1, 1, 1, "rmse_vy");
		
		char str[10];
		sprintf(str, "%.2f s", timestamp/1000000.0);
		string strTemp(str);
		viewer->addText("time: " + strTemp, 30, 175, 20, 1, 1, 1, "timestamp");

		if(timestamp > 1.0e6)
		{

			if(rmse[0] > rmseThreshold[0])
			{
				rmseFailLog[0] = rmse[0];
				pass = false;
			}
			if(rmse[1] > rmseThreshold[1])
			{
				rmseFailLog[1] = rmse[1];
				pass = false;
			}
			if(rmse[2] > rmseThreshold[2])
			{
				rmseFailLog[2] = rmse[2];
				pass = false;
			}
			if(rmse[3] > rmseThreshold[3])
			{
				rmseFailLog[3] = rmse[3];
				pass = false;
			}
		}
		if(!pass)
		{
			viewer->addText("RMSE Failed Threshold", 30, 150, 20, 1, 0, 0, "rmse_fail");
			if(rmseFailLog[0] > 0)
				viewer->addText(" X: "+std::to_string(rmseFailLog[0]), 30, 125, 20, 1, 0, 0, "rmse_fail_x");
			if(rmseFailLog[1] > 0)
				viewer->addText(" Y: "+std::to_string(rmseFailLog[1]), 30, 100, 20, 1, 0, 0, "rmse_fail_y");
			if(rmseFailLog[2] > 0)
				viewer->addText("Vx: "+std::to_string(rmseFailLog[2]), 30, 75, 20, 1, 0, 0, "rmse_fail_vx");
			if(rmseFailLog[3] > 0)
				viewer->addText("Vy: "+std::to_string(rmseFailLog[3]), 30, 50, 20, 1, 0, 0, "rmse_fail_vy");
		}
	}
	
};