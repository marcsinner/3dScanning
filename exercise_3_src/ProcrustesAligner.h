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
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);

		// To apply the pose to point x on shape X in the case of Procrustes, we execute:
		// 1. Translation of a point to the shape Y: x' = x + t
		// 2. Rotation of the point around the mean of shape Y: 
		//    y = R (x' - yMean) + yMean = R (x + t - yMean) + yMean = R x + (R t - R yMean + yMean)
		
		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements
		Matrix4f estimatedPose = Matrix4f::Identity();
		
		for (unsigned int i = 0; i<rotation.rows(); i++){ 
			for (unsigned int j = 0; j<rotation.cols(); j++){ 
				estimatedPose(i,j) = rotation(i,j);
			}
		}

		for (unsigned int j = 0; j<translation.size(); j++){ 
				estimatedPose(j,3) = translation(j);
		}




		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.
		Vector3f mean = Vector3f::Zero();

		for (unsigned int i = 0; i<mean.size(); i++){
			mean = mean+points[i];
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
		//Dynamic instead of "4" did not work - what did i do wrong? MatrixXf also didnt work
		Eigen::Matrix<float,4,3> targetPointsCentered;
		Eigen::Matrix<float,4,3> sourcePointsCentered;



		for (unsigned int i = 0; i<targetPoints.size(); i++){ //sizes are the same targetPoints.size == sourcePoints.size
			//targetCovariance(0) = targetPoints[i] + sourceMean; 
			targetPointsCentered.row(i) = ((targetPoints[i] + targetMean));
			sourcePointsCentered.row(i)=((sourcePoints[i] + sourceMean));
		}
		

		Eigen::Matrix3f covarianceMatrix = Matrix3f::Identity();;

		covarianceMatrix = targetPointsCentered.transpose()*sourcePointsCentered;

		Matrix3f rotation = Matrix3f::Identity();
		JacobiSVD<MatrixXf> jacobi(covarianceMatrix, ComputeThinU | ComputeThinV);

		rotation = jacobi.matrixU() * jacobi.matrixV().transpose(); 

		int a = (int) rotation.determinant();
		Matrix3f mirror;
		mirror << 1,0,0,0,1,0,0,0,-1;

		if (a<0){
			rotation = jacobi.matrixU() *mirror* jacobi.matrixV().transpose();
		}


		return rotation;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, Matrix3f rotation) {
		// TODO: Compute the translation vector from source to target points.
		Vector3f translation = Vector3f::Zero();
		// translation = targetMean - sourceMean;

		translation = -rotation*sourceMean+ targetMean;

		return translation;
	}
};