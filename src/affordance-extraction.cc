//
// Copyright (c) 2016 CNRS
// Authors: Anna Seppala
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/affordance/affordance-extraction.hh>
#include <hpp/affordance/operations.hh>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <algorithm>


namespace hpp {
  namespace affordance {

    BVHModelOBConst_Ptr_t GetModel (FclConstCollisionObjectPtr_t object)
    {
        assert (object->collisionGeometry ()->getNodeType () == fcl::BV_OBBRSS);
        const BVHModelOBConst_Ptr_t model = boost::static_pointer_cast<const BVHModelOB>
                                            (object->collisionGeometry ());
        assert (model->getModelType () == fcl::BVH_MODEL_TRIANGLES);
        return model;
    }

    void searchLinkedTriangles(std::vector<unsigned int>& listPotential, const OperationBasePtr_t& refOp,
                               const std::vector<Triangle>& allTris, std::vector<unsigned int>& searchableTris,
                               const unsigned int& refTriIdx, double& area)
    {
			// TODO: think of a better way of declaring margins?
      const double margin = 1e-15;
			const Triangle& refTri = allTris[refTriIdx];
			searchableTris.erase (std::remove(searchableTris.begin (),
				searchableTris.end (), refTriIdx), searchableTris.end ());
      for (unsigned int searchIdx = 0; searchIdx < allTris.size (); searchIdx++) {
        std::vector<unsigned int>::iterator it = std::find (searchableTris.begin (),
                                                            searchableTris.end (), searchIdx);
          if (it == searchableTris.end ()) {
            continue;
          }
          std::vector<fcl::Vec3f> refPoints;
          refPoints.push_back(refTri.points.p1);
          refPoints.push_back(refTri.points.p2);
          refPoints.push_back(refTri.points.p3);
        for (unsigned int vertIdx = 0; vertIdx < 3; vertIdx++) {
          const Triangle& searchTri = allTris [searchIdx];
          if ((refPoints[vertIdx] - searchTri.points.p1).squaredNorm () < margin
              || (refPoints[vertIdx] - searchTri.points.p2).squaredNorm () < margin
              || (refPoints[vertIdx] - searchTri.points.p3).squaredNorm () < margin) {
            if (refOp->requirement (searchTri.normal)) {
              if ((searchTri.normal - refTri.normal).squaredNorm () < refOp->neighbouringTriangleMargin_) {
                area += searchTri.area;
                listPotential.push_back (searchIdx);
                searchLinkedTriangles (listPotential, refOp, allTris,
                                       searchableTris, searchIdx, area);
              }
            } else {
              // if linked face does not fulfil global requirement, discard
							searchableTris.erase(std::remove(searchableTris.begin(),
								searchableTris.end(), searchIdx), searchableTris.end());
            }
            break; // jump out of vertex loop if triangle already tested for affordance
          }
        }
      }
    }

    SemanticsDataPtr_t affordanceAnalysis (FclConstCollisionObjectPtr_t colObj,
                                           const OperationBases_t& opVec)
    {
      BVHModelOBConst_Ptr_t model =  GetModel (colObj);

      std::vector <Triangle> triangles;
      std::vector <unsigned int> unsetTriangles;
      double totArea = .0;
      std::vector<std::vector<unsigned int> > potentialAffordances (opVec.size ());
      SemanticsDataPtr_t foundAffordances(new SemanticsData());
			foundAffordances->affordances_.resize (opVec.size ());

      for(int i = 0; i < model->num_tris; ++i)
      {
        TrianglePoints tri;
        fcl::Triangle fcltri = model->tri_indices [i];
        tri.p1 = colObj->getRotation () *
          model->vertices [fcltri [0]] + colObj->getTranslation ();
        tri.p2 = colObj->getRotation () *
          model->vertices [fcltri [1]] + colObj->getTranslation ();
        tri.p3 = colObj->getRotation () *
          model->vertices [fcltri [2]] + colObj->getTranslation ();

        triangles.push_back (Triangle (tri));
        // save vector index of triangles and their quantity.
				unsetTriangles.push_back(i);
      }
      std::vector <unsigned int> unseenTriangles;
      for (unsigned int triIdx = 0; triIdx < triangles.size (); triIdx++) {
        // look for triangle in set of triangles not yet given an affordance:
        std::vector<unsigned int>::iterator it = std::find (unsetTriangles.begin (), unsetTriangles.end (), triIdx);
        if (it == unsetTriangles.end ()) {
          continue;
        }
        // set list of searchable (unseen) triangles equal to all triangles not yet given an affordance.
        unseenTriangles = unsetTriangles;
        for (unsigned int opIdx = 0; opIdx < opVec.size (); opIdx++) {
          if (opVec[opIdx]->requirement (triangles[triIdx].normal)) {
             totArea += triangles[triIdx].area;
             potentialAffordances[opIdx].push_back(triIdx);
             searchLinkedTriangles(potentialAffordances [opIdx], opVec[opIdx],
						 	 triangles, unseenTriangles, triIdx, totArea);
            if (totArea > opVec[opIdx]->minArea_) {
              // save totArea for further use as well?
              AffordancePtr_t aff(new Affordance (potentialAffordances [opIdx], colObj));
              foundAffordances->affordances_ [opIdx].push_back (aff);
              for (unsigned int removeIdx = 0; removeIdx < potentialAffordances [opIdx].size (); removeIdx++) {
                std::remove (unsetTriangles.begin (), unsetTriangles.end (), potentialAffordances [opIdx][removeIdx]);
                unsetTriangles.pop_back ();
              }
                // potentialAffordances [opIdx].clear ();
             }
             potentialAffordances [opIdx].clear ();
             totArea = .0;
             break;
          } else if (opIdx >= opVec.size () -1) {
              // delete triangle if it did not fulfil any requirements
              std::remove (unsetTriangles.begin (), unsetTriangles.end (), triIdx);
              unsetTriangles.pop_back ();
            }
        }
      }
      return foundAffordances;
    }

    Eigen::Matrix3d GetRotationMatrix(const Eigen::Vector3d &from, const Eigen::Vector3d &to)
    {
        Eigen::Quaterniond quat; quat.setFromTwoVectors(from, to);
        return quat.toRotationMatrix();
    }

    Eigen::Matrix3d GetRotationMatrix(const std::vector<fcl::Vec3f>& points)
    {
        assert(points.size() > 2);
        fcl::Vec3f z(0.,0.,1.);
        fcl::Vec3f normal = (points[1] - points[0]).cross(points[2] - points[0]);
        return GetRotationMatrix(normal, z);
    }


    typedef  fcl::Vec3f Point;
    typedef  std::vector<Point> T_Point;
    typedef  T_Point::const_iterator CIT_Point;

    std::vector<fcl::Vec3f> projectPoints(const Eigen::Matrix3d& project, const std::vector<fcl::Vec3f>& points)
    {
        std::vector<fcl::Vec3f> res;
        for(CIT_Point cit = points.begin(); cit != points.end(); ++cit)
            res.push_back(project * (*cit));
        return res;
    }

    fcl::Vec3f computeCentroid(const std::vector<fcl::Vec3f>& points)
    {
        fcl::Vec3f acc(0.,0.,0.);
        for(CIT_Point cit = points.begin(); cit != points.end(); ++cit)
            acc += (*cit);
        return acc / points.size();
    }

    std::vector<fcl::Vec3f> translatePoints(const std::vector<fcl::Vec3f>& points, const double reduceSize)
    {
        fcl::Vec3f centroid = computeCentroid(points);
        fcl::Vec3f dir;
        std::vector<fcl::Vec3f> res;
        for(CIT_Point cit = points.begin(); cit != points.end(); ++cit)
        {
            dir = centroid - (*cit); dir.normalize();
            res.push_back((*cit) + dir * reduceSize);
        }
        return res;
    }

    void createAffordanceModel(hpp::affordance::AffordancePtr_t affPtr,
                                                          const std::vector<fcl::Vec3f>& vertices,
                                                          const std::vector<fcl::Triangle>& triangles,
                                                          const unsigned int affIdx,
                                                          std::vector<CollisionObjects_t>& affObjs)
    {
        BVHModelOB_Ptr_t model1 (new BVHModelOB ());
        // add the mesh data into the BVHModel structure
        model1->beginModel ();
        model1->addSubModel (vertices, triangles);
        model1->endModel ();
        // create affordance collision object from created affModel and
        // tranformation of corresponding reference collision object.
        fcl::CollisionObjectPtr_t obj (new fcl::CollisionObject(model1,
                                       affPtr->colObj_->getTransform ()));
        affObjs[affIdx].push_back (obj);
    }

    void addTriangles(hpp::affordance::AffordancePtr_t affPtr, BVHModelOBConst_Ptr_t model,
                      std::vector<std::size_t>& triIndices, const unsigned int triIdx,
                      std::vector<fcl::Vec3f>& vertices, std::vector<fcl::Triangle>& triangles)
    {
        // give triangles of new object new vertex indices (start from 0
        // and go up to 3*nbTris - 1 [all tris have 3 unique indices])
        std::vector<std::size_t> triPoints;
        const fcl::Triangle& refTri = model->tri_indices[affPtr->indices_[triIdx]];
        //     triangles.push_back (refTri);
        for (unsigned int vertIdx = 0; vertIdx < 3; vertIdx++)
        {
            std::vector<std::size_t>::iterator it =
                    std::find (triIndices.begin (), triIndices.end (), refTri[vertIdx]);
            if (it == triIndices.end ())
            {
                    triIndices.push_back (refTri[vertIdx]);
                    vertices.push_back (model->vertices [refTri[vertIdx]]);
                    triPoints.push_back (std::size_t (vertices.size () -1));
            } else
            {
                triPoints.push_back (it - triIndices.begin ());
            }
        }
        if (triPoints.size () != 3)
        {
            std::ostringstream oss("wrong number of vertex indices in triangle!!");
            throw std::runtime_error (oss.str ());
        }
        triangles.push_back (fcl::Triangle (triPoints[0], triPoints[1], triPoints[2]));
    }


    std::vector<CollisionObjects_t> getAffordanceObjects(const SemanticsDataPtr_t& sData)
    {
        std::vector<CollisionObjects_t> affObjs;
        for (unsigned int affIdx = 0; affIdx < sData->affordances_.size (); affIdx ++)
        {
            std::vector<fcl::CollisionObjectPtr_t> objVec;
            affObjs.push_back (objVec);
            // get number of affordances of specific type (lean OR support etc)
            // this corresponds to number of objects to be created for specific aff type
            long unsigned int len = sData->affordances_[affIdx].size ();
            for (unsigned int idx = 0; idx < len; idx++)
            {
                std::vector<fcl::Vec3f> vertices;
                std::vector<fcl::Triangle> triangles;
                std::vector<std::size_t> triIndices;
                hpp::affordance::AffordancePtr_t affPtr = sData->affordances_[affIdx][idx];
                BVHModelOBConst_Ptr_t model =  GetModel (affPtr->colObj_);
                for (unsigned int triIdx = 0; triIdx <  affPtr->indices_.size (); triIdx++)
                    addTriangles(affPtr,model,triIndices,triIdx,vertices,triangles);
                createAffordanceModel(affPtr, vertices, triangles, affIdx,affObjs);
            }
        }
        return affObjs;
    }

//same as getAffordanceObject but apply a reduction of a given size on the mesh
std::vector<CollisionObjects_t> getReducedAffordanceObjects
(const SemanticsDataPtr_t& sData,std::vector<double> reduceSizes)
{
    if(reduceSizes.size() != sData->affordances_.size ()){
        std::ostringstream oss("Error : the vector 'reduceSizes' must have one element per affordance types");
        throw std::runtime_error (oss.str ());
    }
    std::vector<CollisionObjects_t> affObjs;
    affObjs.clear();
    for (unsigned int affIdx = 0; affIdx < sData->affordances_.size (); affIdx ++)
    {
        std::vector<fcl::CollisionObjectPtr_t> objVec;
        affObjs.push_back (objVec);
        // get number of affordances of specific type (lean OR support etc)
        // this corresponds to number of objects to be created for specific aff type
        long unsigned int len = sData->affordances_[affIdx].size ();
        for (unsigned int idx = 0; idx < len; idx++)
        {
            std::vector<fcl::Vec3f> vertices;
            std::vector<fcl::Triangle> triangles;
            std::vector<std::size_t> triIndices;
            hpp::affordance::AffordancePtr_t affPtr = sData->affordances_[affIdx][idx];
            BVHModelOBConst_Ptr_t model =  GetModel (affPtr->colObj_);
            for (unsigned int triIdx = 0; triIdx <  affPtr->indices_.size (); triIdx++)
                addTriangles(affPtr,model,triIndices,triIdx,vertices,triangles);

            if(reduceSizes[affIdx]>std::numeric_limits<double>::epsilon())
            {
                //std::cout<<"For points of affordance : "<<std::endl;
                for(unsigned int id=0;id<vertices.size();++id)
                    std::cout<<"( "<<vertices[id][0]<<" , "<<vertices[id][1]<<" , "<<vertices[id][2]<<" ) "<<std::endl;

                //first we need to sort the vertices in clockwise order.
                // but as we cannot change the order in the 'vertices' vector we use another vector for correspondance of index
                // first getTransformation matrix to project triangles in a 2d plane
                Eigen::Matrix3d project = GetRotationMatrix(vertices);
                std::vector<fcl::Vec3f> projectedVertices = projectPoints(project, vertices);

                // now translate each point by the reducesize factor in the direction
                // towards the centroid
                std::vector<fcl::Vec3f> translatedVertices = translatePoints(projectedVertices, reduceSizes[affIdx]);
                vertices = projectPoints(project.transpose(), translatedVertices);
            }
            createAffordanceModel(affPtr, vertices, triangles, affIdx,affObjs);
        }
    }
    return affObjs;
}

} // namespace affordance
} // namespace hpp


