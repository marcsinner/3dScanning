#include <iostream>
#include <fstream>
#include <array>
#include <math.h>

#include "Eigen.h"

#include "VirtualSensor.h"


struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;

	// color stored as 4 unsigned char
	Vector4uc color;
};


bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = width*height;


	// TODO: Determine number of valid faces
	unsigned nFaces = 0;


	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;
	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices
	for(int i = 0; i < nVertices; i++)
	{
		if (std::isinf(std::abs(vertices[i].position[0])) || std::isnan(std::abs(vertices[i].position[0])) || 
			std::isinf(std::abs(vertices[i].position[1])) || std::isnan(std::abs(vertices[i].position[1])) ||
			std::isinf(std::abs(vertices[i].position[2])) || std::isnan(std::abs(vertices[i].position[2])) ||
			std::isinf(std::abs(vertices[i].position[3])) || std::isnan(std::abs(vertices[i].position[3]))
		)
		{
			outFile << 0 << " " << 0 << " " << 0 << " ";
			outFile << (int)vertices[i].color[0] << " " << (int)vertices[i].color[1] << " " << (int)vertices[i].color[2] << " " << (int)vertices[i].color[3] << std::endl;
		
		}
		else
		{
			outFile << vertices[i].position[0] << " " << vertices[i].position[1] << " " << vertices[i].position[2]  << " ";
			outFile << (int)vertices[i].color[0] << " " << (int)vertices[i].color[1] << " " << (int)vertices[i].color[2] << " " << (int)vertices[i].color[3] << std::endl;
		}
		
	}

	// TODO: save valid faces


	// close file
	outFile.close();

	return true;
}

int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "../../data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();

		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];
		for(int y = 0; y < sensor.GetDepthImageHeight(); y++)
		{
			for(int x = 0; x < sensor.GetDepthImageWidth(); x++)
			{
				int idx = y * sensor.GetDepthImageWidth() + x;
				float depth = depthMap[idx];
				Vector4f position;
				Vector4uc color;

				// check for negative infinity
				if (std::isinf(depth))
				{
					position[0] = depth;
					position[1] = depth;
					position[2] = depth;
					position[3] = depth;

					color[0] = 0;
					color[1] = 0;
					color[2] = 0;
					color[3] = 0;
				}
				else
				{
					color = Vector4uc(colorMap[idx*4+0], colorMap[idx*4+1], colorMap[idx*4+2], colorMap[idx*4+3]);
					float cam_x = ((x - cX) * depth) / fX;
					float cam_y = ((y - cY) * depth) / fY;
					float cam_z = depth;
					position = trajectoryInv * depthExtrinsicsInv * Vector4f(cam_x, cam_y, cam_z, 1.0f);
				}
				vertices[idx].color = color;
				vertices[idx].position = position;
			}
		}

		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}
