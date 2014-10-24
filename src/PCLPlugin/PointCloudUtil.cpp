/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "PointCloudUtil.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/registration/icp.h>
#include <iostream>

using namespace std;
using namespace cnoid;

SgMesh* cnoid::createSurfaceMesh(SgPointSet* pointSet)
{
    if(!pointSet->hasVertices()){
        return 0;
    }

    SgVertexArray* vertices = pointSet->vertices();
    const int numPoints = vertices->size();

    SgMesh* mesh = new SgMesh;
    mesh->setVertices(vertices);

    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    PointCloud::Ptr cloud(new PointCloud(numPoints, 1));
    for(int i=0; i < numPoints; ++i){
        const Vector3f& p = (*vertices)[i];
        cloud->points[i] = pcl::PointXYZ(p.x(), p.y(), p.z());
    }
    
    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    //n.setRadiusSearch(0.03);
    n.setViewPoint(0.0, 0.0, 0.5);
    n.setKSearch(20);
    n.compute(*normals);
    //* normals should not contain the point normals + surface curvatures

    /*
    SgNormalArray& meshNormals = *mesh->setNormals(new SgNormalArray(normals->size()));
    for(size_t i=0; i < normals->size(); ++i){
        meshNormals[i] = Eigen::Map<const Vector3f>(normals->points[i].normal);
    }
    */
    
    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals
    
    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);
    
    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh polygonMesh;
    
    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(0.025);
    
    // Set typical values for the parameters
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4.0); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18.0); // 10 degrees
    gp3.setMaximumAngle(2.0 * M_PI / 3.0); // 120 degrees
    gp3.setNormalConsistency(false);
    gp3.setConsistentVertexOrdering(true);
    
    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(polygonMesh); // The result type is supposed to be triangles

    SgNormalArray& meshNormals = *mesh->setNormals(new SgNormalArray(cloud_with_normals->size()));
    for(size_t i=0; i < cloud_with_normals->size(); ++i){
        meshNormals[i] = Eigen::Map<const Vector3f>(cloud_with_normals->points[i].normal);
    }

    const vector<pcl::Vertices>& triangles = polygonMesh.polygons;
    const int numTriangles = triangles.size();
    mesh->reserveNumTriangles(numTriangles);
    for(int i=0; i < numTriangles; ++i){
        const vector<uint32_t>& triangleVertices = triangles[i].vertices;
        if(triangleVertices.size() == 3){
            mesh->addTriangle(triangleVertices[0], triangleVertices[1], triangleVertices[2]);
        }
    }

    return mesh;
}


bool cnoid::alignPointCloud(SgPointSet* target, SgPointSet* source, Affine3& io_T)
{
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;    

    if(!target->hasVertices() || !source->hasVertices()){
        return false;
    }
    
    const SgVertexArray& targetPoints = *target->vertices();
    PointCloud::Ptr targetCloud(new PointCloud(targetPoints.size(), 1));
    for(size_t i=0; i < targetPoints.size(); ++i){
        const Vector3f& p = targetPoints[i];
        targetCloud->points[i] = pcl::PointXYZ(p.x(), p.y(), p.z());
    }

    const SgVertexArray& sourcePoints = *source->vertices();
    PointCloud::Ptr sourceCloud(new PointCloud(sourcePoints.size(), 1));
    sourceCloud->is_dense = false;
    for(size_t i=0; i < sourcePoints.size(); ++i){
        const Vector3f& p = sourcePoints[i];
        sourceCloud->points[i] = pcl::PointXYZ(p.x(), p.y(), p.z());
    }

    typedef pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> ICP;
    ICP icp;
    icp.setInputTarget(targetCloud);
    icp.setInputSource(sourceCloud);
    pcl::PointCloud<pcl::PointXYZ> Final;
    const ICP::Matrix4 guess(io_T.matrix().cast<ICP::Matrix4::Scalar>());
    icp.align(Final, guess);
    bool hasConverged = icp.hasConverged();
    double score = icp.getFitnessScore();
    //std::cout << "has converged: " << hasConverged << " score: " << score << std::endl;
    io_T = icp.getFinalTransformation().cast<Affine3::Scalar>();
    return hasConverged;
}
