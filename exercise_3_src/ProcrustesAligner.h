#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean);

		// To apply the pose to point x on shape X in the case of Procrustes, we execute:
		// 1. Translation of a point to the shape Y: x' = x + t
		// 2. Rotation of the point around the mean of shape Y: 
		//    y = R (x' - yMean) + yMean = R (x + t - yMean) + yMean = R x + (R t - R yMean + yMean)
		
		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements
		Matrix4f estimatedPose = Matrix4f::Identity();

		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.
		Vector3f mean = Vector3f::Zero();

		for (unsigned int i = 0; i<mean.size(); i++){
			mean(0) = mean(0) + points[i].x();
			mean(1) = mean(1) + points[i].y();
			mean(2) = mean(2) + points[i].z();
		}
		mean(0) = mean(0)/points.size();
		mean(1) = mean(1)/points.size();
		mean(2) = mean(2)/points.size();

		return mean;
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		// Important: The covariance matrices should contain mean-centered source/target points.
		Vector3f translation = computeTranslation(sourceMean, targetMean);

		std::vector<Vector3f> targetPointsCentered;
		std::vector<Vector3f> sourcePointsCentered;


		for (unsigned int i = 0; i<targetPoints.size(); i++){ //sizes are the same targetPoints.size == sourcePoints.size
			//targetCovariance(0) = targetPoints[i] + sourceMean; 
			targetPointsCentered.push_back((targetPoints[i] + sourceMean));
			sourcePointsCentered.push_back((sourcePoints[i] + sourceMean));
		}


		//TODO: fill matrix (from target/sourcePointsCentered)
		Eigen::Matrix3f targetCovariance;
		Eigen::Matrix3f sourceCovariance;



		Matrix3f rotation = Matrix3f::Identity();
		JacobiSVD<Matrix3f> jacobi(targetCovariance.transpose() * sourceCovariance);
		rotation = jacobi.matrixU() * jacobi.matrixV().transpose(); 

		

		return rotation;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean) {
		// TODO: Compute the translation vector from source to target points.
		Vector3f translation = Vector3f::Zero();
		translation = targetMean - sourceMean;

		return translation;
	}
};