#include <RKD/simPointCloud.h>

using namespace RKD;

void simPointCloud::removeNaNFromPointCloud (const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out, std::vector<int> &index)
 {
   // If the clouds are not the same, prepare the output
   if (&cloud_in != &cloud_out)
   {
     cloud_out.header = cloud_in.header;
     cloud_out.points.resize (cloud_in.points.size ());
     cloud_out.sensor_origin_ = cloud_in.sensor_origin_;
     cloud_out.sensor_orientation_ = cloud_in.sensor_orientation_;
   }
   // Reserve enough space for the indices
   index.resize (cloud_in.points.size ());
 
   // If the data is dense, we don't need to check for NaN
   if (cloud_in.is_dense)
   {
     // Simply copy the data
     cloud_out = cloud_in;
     for (std::size_t j = 0; j < cloud_out.points.size (); ++j)
       index[j] = static_cast<int>(j);
   }
   else
   {
     std::size_t j = 0;
     for (std::size_t i = 0; i < cloud_in.points.size (); ++i)
     {
       if (!std::isfinite (cloud_in.points[i].x) ||
           !std::isfinite (cloud_in.points[i].y) ||
           !std::isfinite (cloud_in.points[i].z))
         continue;
       cloud_out.points[j] = cloud_in.points[i];
       index[j] = static_cast<int>(i);
       j++;
     }
     if (j != cloud_in.points.size ())
     {
       // Resize to the correct size
       cloud_out.points.resize (j);
       index.resize (j);
     }
 
     cloud_out.height = 1;
     cloud_out.width  = static_cast<std::uint32_t>(j);
 
     // Removing bad points => dense (note: 'dense' doesn't mean 'organized')
     cloud_out.is_dense = true;
   }
 }



bool simPointCloud::loadPCD(const std::string& pointcloud_path, const Eigen::Matrix4f transf){

	// Declare a Point Cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	
	// Get the Point Cloud from the file
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (pointcloud_path.c_str(), *cloud_in) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file %s\n", pointcloud_path);
		return false;
	}

 	std::vector<int> indices;

	// Remove Indices
	removeNaNFromPointCloud(*cloud_in, *cloudPtr_, indices);

	// Apply transformation
	pcl::transformPointCloud (*cloudPtr_, *cloudPtr_, transf);
  
	// Define PointSource structure
	pointSource_ = vtkSmartPointer<vtkPointSource>::New();
	pointSource_->SetNumberOfPoints(cloudPtr_->width * cloudPtr_->height);
	pointSource_->SetRadius(1.0);
	pointSource_->Update();

	vtkPolyData* output = vtkPolyData::SafeDownCast(pointSource_->GetOutput());

	vtkPoints* newPoints = vtkPoints::New();
	newPoints->SetDataType(VTK_DOUBLE);
	newPoints->Allocate(cloudPtr_->width * cloudPtr_->height);

	// Load Point Cloud in the VTK point source
	double newPoint[3];

	for(const auto &point: cloudPtr_->points){

		newPoint[0] = point.x;
		newPoint[1] = point.y;
		newPoint[2] = point.z;

		newPoints->InsertNextPoint(newPoint);
	}

	output->SetPoints(newPoints);
	newPoints->Delete();


	// Create mapper
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(pointSource_->GetOutputPort());

	// Connect to the an Actor
	PCLActor_ = vtkSmartPointer<vtkActor>::New();
	PCLActor_->SetMapper(mapper);

	return true;
}
