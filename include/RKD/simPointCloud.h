// Author: Tommaso Pardi
// Date: 07/03/2020

#ifndef _simPointCloud_header_
#define _simPointCloud_header_

// PCL
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
// VTK
#include <vtkActor.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkProperty.h>
#include <vtkPointSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>

namespace RKD{

class simPointCloud{

public:

	simPointCloud():cloudPtr_(new pcl::PointCloud<pcl::PointXYZ>){}
	~simPointCloud(){}
	
	// Remove NaN from PointCloud
	void removeNaNFromPointCloud (const pcl::PointCloud<pcl::PointXYZ> &, pcl::PointCloud<pcl::PointXYZ> &, std::vector<int> &);
	bool loadPCD(const std::string&, const Eigen::Matrix4f);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr_;	
	vtkSmartPointer<vtkPointSource> pointSource_;
	vtkSmartPointer<vtkActor> PCLActor_;
};
}


#endif