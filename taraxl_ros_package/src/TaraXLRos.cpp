#include "TaraXLRos.h"

float maxDisp = 64;

void taraxlros::rosPublish ()
{

	status = cam.enumerateDevices (camList);

	if (status != TARAXL_SUCCESS) 
	{
		cout << "Camera enumeration failed" << endl;
		exit (0);
	}


	if (camList.size ()==0)
	{
		cout << "No cameras connected" << endl;
		exit (0);
	}

	taraxlCam = camList.at (0);

	status = taraxlCam.connect ();


	if (status != TARAXL_SUCCESS) 
	{
		cout << "Camera connect failed" << endl;
		exit (0);
	}


	taraxlDepth = new TaraXLDepth (taraxlCam);

	if (taraxlDepth == NULL) 
	{
		cout << "Unable to create instance to TaraDepth" << endl;
    		exit (0);
	}	

	ROS_INFO ("TaraXL ROS running");


	image_transport::ImageTransport itTaraXL (nodeHandle);

	//openCV Mat objects
	Mat left, right, grayDisp, depthMap;	


	//ROS messages
	sensor_msgs::ImagePtr leftMsg;
	sensor_msgs::ImagePtr rightMsg;
	sensor_msgs::ImagePtr depthMsg;
	stereo_msgs::DisparityImagePtr disparityMsg = boost::make_shared<stereo_msgs::DisparityImage> ();
	
	//added cameraInfo msg
	sensor_msgs::CameraInfoPtr leftCam = boost::make_shared<sensor_msgs::CameraInfo> ();
	sensor_msgs::CameraInfoPtr rightCam = boost::make_shared<sensor_msgs::CameraInfo> ();

	//publishers-advertise to topics
	pubLeft = itTaraXL.advertise ("left/image_rect", 1);
	pubRight = itTaraXL.advertise ("right/image_rect", 1);
	pubDisparity = nodeHandle.advertise<stereo_msgs::DisparityImage> ("stereo/disparity/image", 1);
	pubDepth = itTaraXL.advertise ("depth/image", 1);

	pubCamInfoLeft = nodeHandle.advertise<sensor_msgs::CameraInfo>("/taraxl/left/camera_info", 1);
	pubCamInfoRight = nodeHandle.advertise<sensor_msgs::CameraInfo>("/taraxl/right/camera_info", 1);


	//Dynamic Reconfiguration
	dynamic_reconfigure::Server<taraxl_ros_package::taraxlrosConfig> server;	
	dynamic_reconfigure::Server<taraxl_ros_package::taraxlrosConfig>::CallbackType settings;
    	settings = boost::bind (&taraxlros::dynamicReconfCallback, this, _1, _2);
   	server.setCallback (settings);
	
	while (ros::ok ())
	{

		//Get number of subscribers
		int dispImgSuber = pubDisparity.getNumSubscribers ();
		int depthImgSuber = pubDepth.getNumSubscribers ();
		int leftImgSuber = pubLeft.getNumSubscribers ();
		int rightImgSuber = pubRight.getNumSubscribers ();


		//Obtain and publish disparity and depth image if both are subscribed
		if ( dispImgSuber >0 && depthImgSuber > 0 )
		{
			
			status = taraxlDepth->getMap (left, right, grayDisp, true, depthMap, true);  //SDK function to obtain Disparity image

			if (status != TARAXL_SUCCESS) 
			{
				cout << "Failed to get Disparity and depth image" << endl;
				exit (0);
			}

	
			//Publish disparity image
			disparityPublisher(disparityMsg, grayDisp);
			pubDisparity.publish (disparityMsg);  

			//Publish depth map		
			depthMap.convertTo (depthMap, CV_8UC1);
			imagePublisher (depthMsg, depthMap);
			pubDepth.publish (depthMsg); 

		}

		//Obtain and publish disparity image if subscribed
		else if (dispImgSuber > 0)
		{
			
			status = taraxlDepth->getMap (left, right, grayDisp, true, depthMap, false);  //SDK function to obtain Disparity image

			if (status != TARAXL_SUCCESS) 
			{
				cout << "Failed to get Disparity image" << endl;
				exit (0);
			}

			//Publish disparity image
			disparityPublisher(disparityMsg, grayDisp);
			pubDisparity.publish (disparityMsg);  
		}

		//Obtain and publish depth image if subscribed
		else if (depthImgSuber > 0)
		{

			status = taraxlDepth->getMap (left, right, grayDisp, false, depthMap, true);  //SDK function to obtain Depth image

			if (status != TARAXL_SUCCESS) 
			{ 

				cout << "Failed to get Depth image" << endl;
				exit (0);
			}

			//Publish depth map
			depthMap.convertTo (depthMap, CV_8UC1);
			imagePublisher (depthMsg, depthMap);
			pubDepth.publish (depthMsg);  
			
		}
		
		//Obtain and publish left/right image if subscribed
		if (leftImgSuber > 0 || rightImgSuber > 0)
		{

			status = taraxlDepth->getMap (left, right, grayDisp, false, depthMap, false);  //SDK function to obtain left and right image

			if (status != TARAXL_SUCCESS) 
			{
				cout << "Failed to get left and right images" << endl;
				exit (0);
			}

			//Publish left and right image
			imagePublisher (leftMsg,left);
			imagePublisher (rightMsg,right);
			pubLeft.publish (leftMsg);    
			pubRight.publish (rightMsg);
			
			cameraInfoLeftPublisher(leftCam);
		    pubCamInfoLeft.publish(leftCam);
		    cameraInfoRightPublisher(rightCam);
		    pubCamInfoRight.publish(rightCam);
			
		}

		ros::spinOnce ();
			
	}

}	


void taraxlros::imagePublisher(sensor_msgs::ImagePtr &imageMsg, Mat image)
{
	
	int num = 1; // for endianness detection
	
	//Convert to ROS message
	imageMsg = cv_bridge::CvImage (std_msgs::Header (), "mono8", image).toImageMsg ();	
	if (imageMsg == NULL) 
	{
		cout << "Failed to get image "<< endl;
		exit (0);
	}

	imageMsg->header.stamp = ros::Time::now ();
	imageMsg->header.frame_id = "taraxl";
	imageMsg->is_bigendian = ! (* (char*)&num == 1);

	
}


void taraxlros::disparityPublisher(stereo_msgs::DisparityImagePtr &dispMsg, Mat dispImage)
{

	int num = 1; // for endianness detection

	//Convert to ROS message
	dispMsg->min_disparity = 0;
	dispMsg->max_disparity = maxDisp;
	dispMsg->header.stamp = ros::Time::now ();
    	dispMsg->header.frame_id = "taraxl";
	sensor_msgs::Image& dimage = dispMsg->image;
	dimage.width  = dispImage.size ().width ;
	dimage.height = dispImage.size ().height ;
	dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
	dimage.step = dimage.width * sizeof (float);
	dimage.data.resize (dimage.step * dimage.height);
	cv::Mat_<float> dmat (dimage.height, dimage.width, reinterpret_cast<float*> (&dimage.data[0]), dimage.step);
	dispImage.convertTo (dmat, dmat.type ());

	dispMsg->image.header = dispMsg->header;
	dispMsg->image.is_bigendian = ! (* (char*)&num == 1);

	dispMsg->valid_window.x_offset = 0;
	dispMsg->valid_window.y_offset = 0;
	dispMsg->valid_window.width    = 0;
	dispMsg->valid_window.height   = 0;
	dispMsg->T                     = 0.06;
	dispMsg->f                     = 7.0966755474656929e+002;
	dispMsg->delta_d               = 0;

	if (dispMsg == NULL) 
	{
		cout << "Failed to get disparity image "<< endl;
  		exit (0);
	}

}	

void taraxlros::cameraInfoLeftPublisher(sensor_msgs::CameraInfoPtr &ci){
	ci->width = 752;
	ci->height = 480;
	float k_temp[] = { (float)716.23767806883029, 0., (float)380.77949374262056, 0.,
       (float)716.23767806883029, (float)217.86685878818076, 0., 0., 1. };
	
	for(int i = 0; i < 9; i ++){
		ci->K[i] = k_temp[i];
	}

	float d_temp[] = { (float)0.075142546188962184, (float)-0.093787546047981382,
       (float)0.0016692040534575499, (float)-0.00040889004262752079};

	
    for(int i = 0; i < 4; i++){
		ci->D.push_back(d_temp[i]);
	}
	
	float r_temp[] = {(float)0.99863313595266778, (float)-0.0036214326873450909,
       (float)-0.052141586115417193, (float)0.0036833309798456879,
       (float)0.99999262128647204, (float)0.0010910753884294238,
       (float)0.052137250121514329, (float)-0.0012816387561852772,
       (float)0.99863910626004693};

	for(int i = 0; i < 9; i ++){
		ci->R[i] = r_temp[i];
	}

    float p_temp[] = { (float)765.60556152317668, 0., (float)425.82975387573242, 0., 0.,
       (float)765.60556152317668, (float)222.25554084777832, 0., 0., 0., 1.,
       0. };
    
	for(int i = 0; i < 12; i ++){
		ci->P[i] = p_temp[i];
	}

    ci->distortion_model = "equidistant";
}

void taraxlros::cameraInfoRightPublisher(sensor_msgs::CameraInfoPtr &ci){
	ci->width = 752;
	ci->height = 480;
	float k_temp[] = { (float)716.23767806883029, 0., (float)379.52224231819093, 0.,
       (float)716.23767806883029, (float)226.65882878276798, 0., 0., 1. };
	
	for(int i = 0; i < 9; i ++){
		ci->K[i] = k_temp[i];
	}

	float d_temp[] = {(float)0.070940303672828275, (float)-0.083481074716348150,
       (float)-0.0030675339642308669, (float)1.0042332230591974};

	//ci->D = new float(4);

    for(int i = 0; i < 4; i++){
		ci->D.push_back(d_temp[i]);
	}
	
	float r_temp[] = { (float)0.99884162841721502, (float)-0.0053589610059405256,
       (float)-0.047819273078780991, (float)0.0053129464379608748,
       (float)0.99998529289134919, (float)-0.0010893122370687993,
       (float)0.047824407377337952, (float)0.00083398917215879236,
       (float)0.99885541021764856 };

	for(int i = 0; i < 9; i ++){
		ci->R[i] = r_temp[i];
	}

    float p_temp[] = { (float)765.60556152317668, 0., (float)425.82975387573242,
       (float)-46009.434918815954, 0., (float)765.60556152317668,
       (float)222.25554084777832, 0., 0., 0., 1., 0. };
    
	for(int i = 0; i < 12; i ++){
		ci->P[i] = p_temp[i];
	}

    ci->distortion_model = "equidistant";
}


void taraxlros::dynamicReconfCallback (taraxl_ros_package::taraxlrosConfig &config, uint32_t level)
{
  
	enum SETTINGS {BRIGHTNESS, EXPOSURE, ACCURACY, AUTOEXPOSURE};

	switch (level)
	{
	
		case BRIGHTNESS:
			status = taraxlCam.setBrightness (config.brightness);
			break;

		case EXPOSURE:
			status = taraxlCam.setExposure (config.exposure);
			config.autoExposure = false;
			break;

		case ACCURACY:
			if (config.accuracy == 0) 
			{
				status = taraxlDepth->setAccuracy (TaraXLSDK::HIGH);
			}
			else if (config.accuracy == 1) 
			{	
				status = taraxlDepth->setAccuracy (TaraXLSDK::LOW);
			}
			maxDisp = (config.accuracy == 1) ? 64 : 128 ;
			break;

		case AUTOEXPOSURE:
			if (config.autoExposure)
				status = taraxlCam.enableAutoExposure ();
			break;
	}

}


int main (int argc, char **argv)
{
	
	ros::init (argc, argv, "taraxl_ros_package");
	 
	taraxlros roswrapper;
		
	roswrapper.rosPublish ();

	return 0;
	

}
