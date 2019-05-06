#include <iostream>
#include <fstream> 
#include <string.h>
#include <algorithm>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>  
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#define pi 3.1415926
using namespace cv;
using namespace std;
using namespace pcl::console;
int MaxIndex(int num[],int length)
{
	int i,max,max_sign;
	max=num[0];
	for(i=1;i<length;i++)
	{
      	  if(num[i]>max)
      	  {
        	 max=num[i];
       	    	 max_sign=i;
      	  }
    	}
    return max_sign;
}

float Histogram(float array[],int bin,int size)
{
	int *num= new int[bin];
	float result;
	for(int i=0;i<bin;i++)
		num[i]=0;
	float maxValue,minValue,distance;
	maxValue = *max_element(array, array + size);
	minValue = *min_element(array, array + size);
        distance = (maxValue - minValue)/bin; 
	//cout<<"minValue ="<<minValue<<endl<<"maxValue ="<<maxValue<<endl<<"distance ="<<distance<<endl;
	for(int i=0;i<size;i++)
	{
		for(int j=0;j<bin;j++)
			if(array[i] >= (minValue+j*distance) && array[i] < (minValue+(j+1)*distance))
				{num[j]++;break;}	
	}//统计各分段的频数
	num[bin-1]++;
 	//for(int i=0;i<bin;i++) cout<<num[i]<<" "; 
	result=minValue+(MaxIndex(num,bin)+0.5)*distance;//计算直方图最高处的中点值
	return result;			
}

float DirecAngle(float y,float x)
{
   if(y<0&&x>0) return atan(y/x)+2*pi;
   if(y>0&&x>0) return atan(y/x);
   return atan(y/x)+pi;
}

int    default_k = 0,default_iter=1,default_bin=20;
double x=0,y=0,z=0;
int main(int argc,char** argv)
{
/*
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("car0.pcd", *cloud)==-1)
    {
        PCL_ERROR("Could not read file\n");
    }
*/
    int k = default_k,iter=default_iter,bin=default_bin;
    parse_argument (argc, argv, "-k", k);
    parse_argument (argc, argv, "-i", iter);
	parse_argument (argc, argv, "-x", x);
	parse_argument (argc, argv, "-y", y);
        parse_argument (argc, argv, "-z", z);
    vector<int> p_file_indices;
    p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
    if (p_file_indices.size () != 2)
    {
      print_error ("Need one input PCD file and one output PCD file to continue.\n");
      return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[p_file_indices[0]], *cloud);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);

    //点云法向计算时，搜索近邻点数量
    n.setKSearch(k);

    n.compute(*normals);
    int sizeCloud=cloud->points.size();
    float XOY[sizeCloud][2],YOZ[sizeCloud][2],XOZ[sizeCloud][2];
    float Rz[sizeCloud],Rx[sizeCloud],Ry[sizeCloud];
    for(size_t i=0;i < normals->points.size (); ++i)
	{
		 XOY[i][0]=normals->points[i].normal_x;
		 XOY[i][1]= normals->points[i].normal_y;

		 YOZ[i][0]=normals->points[i].normal_y;
		 YOZ[i][1]=normals->points[i].normal_z;

		 XOZ[i][0]= normals->points[i].normal_x;
		 XOZ[i][1]= normals->points[i].normal_z;
	}
   for(size_t i=0;i < normals->points.size (); ++i)
   	{
		Rz[i]= DirecAngle(XOY[i][1],XOY[i][0]);
		Rx[i]= DirecAngle(YOZ[i][1],YOZ[i][0]);
   		Ry[i]= DirecAngle(XOZ[i][1],XOZ[i][0]);
	}
   
   int size=normals->points.size ();
	float rz,ry,rx;
  // cout<<"size:"<<size<<endl;
   	rz=Histogram(Rz,bin,size);
	rx=Histogram(Rx,bin,size);
	ry=Histogram(Ry,bin,size);
	cout<<rz<<" "<<rx<<" "<<ry<<endl<<endl;


//对第二张图片提取法向量
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>(argv[p_file_indices[1]], *cloud1);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n1;
	pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);


	Eigen::Matrix4d RZ= Eigen::Matrix4d::Identity();
	Eigen::Matrix4d RY= Eigen::Matrix4d::Identity();
	Eigen::Matrix4d RX= Eigen::Matrix4d::Identity();
	Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
 



	pcl::io::loadPCDFile<pcl::PointXYZ>(argv[p_file_indices[1]], *cloud1);
	int sizeCloud1=cloud1->points.size();

	float XOY1[sizeCloud1][2],YOZ1[sizeCloud1][2],XOZ1[sizeCloud1][2];
	float Rz1[sizeCloud1],Rx1[sizeCloud1],Ry1[sizeCloud1];

	float rz1,ry1,rx1;
	float yaw,pitch,roll;

	int I=0;
while(I<iter)
{

    tree1->setInputCloud(cloud1);
    n1.setInputCloud(cloud1);
    n1.setSearchMethod(tree1);
    n1.setKSearch(k);
    n1.compute(*normals1);
//z轴转角yaw
    for(size_t i=0;i < normals1->points.size (); ++i)
	{
		 XOY1[i][0]=normals1->points[i].normal_x;
		 XOY1[i][1]= normals1->points[i].normal_y;
	}
    for(size_t i=0;i < normals1->points.size (); ++i)
   	{
		Rz1[i]= DirecAngle(XOY1[i][1],XOY1[i][0]);
	}
	rz1=Histogram(Rz1,bin,sizeCloud1);

	if(rz>rz1) yaw=rz-rz1;// 目标点云角度大于源点云，则旋转角为正，可以直接作差
 		else yaw=2*pi-(rz1-rz);//否则计算角度差的周角补角

//	yaw=rz-rz1;
//	if(abs(rz1-rz)<0.05) yaw=0;//旋转角度小于pi/60=3度时，视作无旋转

	RZ(0,0)=cos(yaw);
	RZ(0,1)=-sin(yaw);
	RZ(1,0)=sin(yaw);
	RZ(1,1)=cos(yaw);

	R *= RZ;
 	
	pcl::transformPointCloud(*cloud1, *cloud1, RZ);
//x轴旋转roll
  tree1->setInputCloud(cloud1);
    n1.setInputCloud(cloud1);
    n1.setSearchMethod(tree1);
    n1.setKSearch(k);
    n1.compute(*normals1);
    	for(size_t i=0;i < normals1->points.size (); ++i)
	{
		 YOZ1[i][0]=normals1->points[i].normal_y;
		 YOZ1[i][1]=normals1->points[i].normal_z;
	}    

   	for(size_t i=0;i < normals1->points.size (); ++i)
   	{
		Rx1[i]= DirecAngle(YOZ1[i][1],YOZ1[i][0]);
	}

	rx1=Histogram(Rx1,bin,sizeCloud1);

	if(rx>rx1) roll=rx-rx1;
		else roll=2*pi-(rx1-rx);

//	roll=rx-rx1;
//	if(abs(rx1-rx)<0.05) roll=0;
	
	RX(1,1)=cos(roll);
	RX(1,2)=-sin(roll);
	RX(2,1)=sin(roll);
	RX(2,2)=cos(roll);

	R *= RX;
 	
	pcl::transformPointCloud(*cloud1, *cloud1, RX);
//y轴旋转pitch

  tree1->setInputCloud(cloud1);
    n1.setInputCloud(cloud1);
    n1.setSearchMethod(tree1);
    n1.setKSearch(k);
    n1.compute(*normals1);
	for(size_t i=0;i < normals1->points.size (); ++i)
	{
		 XOZ1[i][0]= normals1->points[i].normal_x;
		 XOZ1[i][1]= normals1->points[i].normal_z;
	}
   	for(size_t i=0;i < normals1->points.size (); ++i)
   	{
   		Ry1[i]= DirecAngle(XOZ1[i][1],XOZ1[i][0]);
	}
	ry1=Histogram(Ry1,bin,sizeCloud1);

	if(ry>ry1) pitch=ry-ry1;
		else pitch=2*pi-(ry1-ry);


//	pitch=ry-ry1;
//	if(abs(ry1-ry)<0.05) pitch=0;
	
	RY(0,0)=cos(pitch);
	RY(0,2)=sin(pitch);
	RY(2,0)=-sin(pitch);
	RY(2,2)=cos(pitch);

	R *= RY;
 	
	pcl::transformPointCloud(*cloud1, *cloud1, RY);
	//cout<<"No."<<I+1<<":"<<endl<<rz1<<" "<<rx1<<" "<<ry1<<endl<<endl;

	cout<<rz1<<" "<<rx1<<" "<<ry1<<endl<<endl;

	I++;
}
/*
//projection得到的平移向量	
	R1(0,3)=-0.0542191;
	R1(1,3)=0.0848129;
	R1(2,3)=0.223845;
*/

	R(0,3)=x;
	R(1,3)=y;
	R(2,3)=z;

	cout<<"Final R ="<<endl<<R<<endl;

	//cout<<"Final R，T ="<<endl<<R1<<endl;

//图形显示模块
    pcl::visualization::PCLVisualizer viewer;


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>(argv[p_file_indices[1]], *cloud2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> incloudHandler(cloud2, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outcloudHandler(cloud, 225, 30, 30);

	int v1 = 1;
    viewer.createViewPort(0, 0, 0.5, 1, v1);
    viewer.createViewPortCamera(v1);
    viewer.addPointCloud(cloud2, incloudHandler, "In",v1);
    viewer.addPointCloud(cloud, outcloudHandler, "Out",v1);
    viewer.addCoordinateSystem(0.1, "cloud", v1);
 
    int v2 = 1;
    viewer.createViewPort(0.5, 0, 1, 1, v2);
    viewer.createViewPortCamera(v2);
    viewer.setCameraFieldOfView(0.785398,v2);
    viewer.setBackgroundColor(0.0, 0.2, 1.0,v2);
    viewer.setCameraPosition(
        0, 0, 0,
        0, 0, -1,
        0, 0, 0,v2);

	pcl::transformPointCloud(*cloud2, *cloudOut, R);

 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> incloudHandler1(cloudOut, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outcloudHandler1(cloud, 225, 30, 30);
   // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud1cloudHandler1(cloud1, 0, 255, 0);
    //viewer.addPointCloud(cloud, incloudHandler1, "In22",v2);
    viewer.addPointCloud(cloudOut, incloudHandler1, "Out22",v2);
    viewer.addPointCloud(cloud, outcloudHandler1, "Final22",v2);
    viewer.addCoordinateSystem(0.1, "cloud22", v2);
 
    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
 
/*
	if(rz1>rz) yaw=rz1-rz;
 		else yaw=2*pi-(rz-rz1);
	if(ry1>ry) pitch=ry1-ry;
		else pitch=2*pi-(ry-ry1);
	if(rx1>rx) roll=rx1-rx;
		else roll=2*pi-(rx-rx1);


	if(rz>rz1) yaw=rz-rz1;// 目标点云角度大于源点云，则旋转角为正，可以直接作差
 		else yaw=2*pi-(rz1-rz);//否则计算角度差的周角补角
	if(ry>ry1) pitch=ry-ry1;
		else pitch=2*pi-(ry1-ry);
	if(rx>rx1) roll=rx-rx1;
		else roll=2*pi-(rx1-rx);

	if(abs(rz1-rz)<0.1) yaw=0;//旋转角度小于pi/30=6度时，视作无旋转
	if(abs(rx1-rx)<0.1) pitch=0;
	if(abs(ry1-ry)<0.1) roll=0;
	
	RZ(0,0)=cos(yaw);
	RZ(0,1)=-sin(yaw);
	RZ(1,0)=sin(yaw);
	RZ(1,1)=cos(yaw);
	RY(0,0)=cos(pitch);
	RY(0,2)=sin(pitch);
	RY(2,0)=-sin(pitch);
	RY(2,2)=cos(pitch);
	RX(1,1)=cos(roll);
	RX(1,2)=-sin(roll);
	RX(2,1)=sin(roll);
	RX(2,2)=cos(roll);

	
	rotation_matrix = RZ*RY*RX;
	cout<<"rotation_matrix = "<<endl<<rotation_matrix<<endl<<endl;
	rotation_matrix1.block<3,3>(0,0) =rotation_matrix.block<3,3>(0,0);
   	pcl::transformPointCloud(*cloud1, *cloud1, rotation_matrix1);
	cout<<"rotation_matrix1 = "<<endl<<rotation_matrix1<<endl<<endl;
	R=R*rotation_matrix;
	R1=R1*rotation_matrix1;

*/
/*
	if (I==0) 
	{
		pcl::io::loadPCDFile<pcl::PointXYZ>(argv[p_file_indices[1]], *cloud1);
		
		Eigen::MatrixXd Q(sizeCloud1,3);
		for(size_t i=0;i < sizeCloud1; ++i)
	   	{
			Q(i,0)=cloud1->points[i].x;
			Q(i,1)=cloud1->points[i].y;
			Q(i,2)=cloud1->points[i].z;
		}	
	}
	else
	{
		for (size_t i = 0; i < sizeCloud1; ++i)
		{
			cloud1->points[i].x = Q(i,0);
			cloud1->points[i].y = Q(i,1);
			cloud1->points[i].z = Q(i,2);
		}
	}

*/	


   // pcl::io::loadPCDFile<pcl::PointXYZ>(argv[p_file_indices[1]], *cloud1);



  //  system("pause");
	//输出测试txt文件
	
//cout<<"Final rotation_matrix = "<<endl<<R<<endl<<endl;
      //  cout<<rotation_matrix<<endl;
/*
    ofstream Angleout("DirecAngleCar22_RT.txt");
    for(size_t i=0;i < normals->points.size (); ++i)
   	{
		Angleout<<Rz[i]<<" "<<Rx[i]<<" "<<Ry[i]<<endl;
	}
    Angleout.close();

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);  
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);  
    //pcl::io::savePCDFileASCII("car00.pcd", *cloud);
    pcl::io::savePCDFileASCII(argv[p_file_indices[1]], *cloud_with_normals);

*/
    return 0;
}

