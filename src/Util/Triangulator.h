/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_TRIANGULATOR_H
#define CNOID_UTIL_TRIANGULATOR_H

#include <vector>

namespace cnoid {

template<class TVector3Array> class Triangulator
{
    typedef typename TVector3Array::value_type TVector3;
        
    enum Convexity { FLAT, CONVEX, CONCAVE };
        
    const TVector3Array* vertices;
    const std::vector<int>* orgPolygon;                                                                  
    std::vector<int> triangles_;
    std::vector<int> workPolygon;
    TVector3 ccs; // cyclic cross sum
    std::vector<char> earMask;

    const TVector3& vertex(int localIndex)
        {
            return (*vertices)[(*orgPolygon)[localIndex]];
        }

    const TVector3& workVertex(int workPolygonIndex)
        {
            return (*vertices)[(*orgPolygon)[workPolygon[workPolygonIndex]]];
        }

    Convexity calcConvexity(int ear)
        {
            int n = workPolygon.size();

            const TVector3& p0 = workVertex((ear + n - 1) % n);
            TVector3 a = workVertex(ear) - p0;
            TVector3 b = workVertex((ear + 1) % n) - p0;
            TVector3 ccs = a.cross(b);
            
            Convexity convexity;
            
            if((ccs.norm() / (a.norm() + b.norm())) < 1.0e-4f){
                convexity = FLAT;
            } else {
                convexity = (this->ccs.dot(ccs) > 0.0f) ? CONVEX : CONCAVE;
            }
            
            return convexity;
        }

    bool checkIfEarContainsOtherVertices(int ear)
        {
            bool contains = false;

            const int n = workPolygon.size();

            if(n > 3){
                const int prev = (ear + n -1) % n;
                const int next = (ear+1) % n;
                const TVector3& a = workVertex(prev);
                const TVector3& b = workVertex(ear);
                const TVector3& c = workVertex(next);

                earMask[prev] = earMask[ear] = earMask[next] = 1;

                for(size_t i=0; i < workPolygon.size(); ++i){
                    if(!earMask[i]){
                        const TVector3& p = workVertex(i);
                        if(((a - p).cross(b - p)).dot(ccs) <= 0.0f){
                            continue;
                        }
                        if(((b - p).cross(c - p)).dot(ccs) <= 0.0f){
                            continue;
                        }
                        if(((c - p).cross(a - p)).dot(ccs) <= 0.0f){
                            continue;
                        }
                        contains = true;
                        break;
                    }
                }

                earMask[prev] = earMask[ear] = earMask[next] = 0;
            }

            return contains;
        }

public:
    void setVertices(const TVector3Array& vertices)
        {
            this->vertices = &vertices;
        }

    /**
       @return The number of triangles
    */
    int apply(const std::vector<int>& polygon)
        {
            triangles_.clear();

            size_t numOrgVertices = polygon.size();

            if(numOrgVertices > earMask.size()){
                earMask.resize(numOrgVertices, 0);
            }
            
            if(numOrgVertices < 3){
                return 0;
            } else if(numOrgVertices == 3){
                triangles_.push_back(0);
                triangles_.push_back(1);
                triangles_.push_back(2);
                return 1;
            }
            
            orgPolygon = &polygon;

            workPolygon.resize(numOrgVertices);
            for(size_t i=0; i < numOrgVertices; ++i){
                workPolygon[i] = i;
            }
            
            ccs.setZero();
            const TVector3& o = vertex(0);
            for(size_t i=1; i < numOrgVertices - 1; ++i){
                ccs += (vertex(i) - o).cross(vertex((i+1) % numOrgVertices) - o);
            }

            int numTriangles = 0;

            while(true) {
                int n = workPolygon.size();
                if(n < 3){
                    break;
                }
                int target = -1;
                for(int i=0; i < n; ++i){
                    Convexity convexity = calcConvexity(i);
                    if(convexity == FLAT){
                        target = i;
                        break;
                    } else if(convexity == CONVEX){
                        if(!checkIfEarContainsOtherVertices(i)){
                            triangles_.push_back(workPolygon[(i + n - 1) % n]);
                            triangles_.push_back(workPolygon[i]);
                            triangles_.push_back(workPolygon[(i + 1) % n]);
                            target = i;
                            numTriangles++;
                            break;
                        }
                    }
                }
                if(target < 0){
                    break;
                }
                for(int i = target + 1; i < n; ++i){
                    workPolygon[target++] = workPolygon[i];
                }
                workPolygon.pop_back();
            }
    
            return numTriangles;
        }

    /**
       Triangulated indices.
       This value is available after calling the 'apply' method.
       The indices are local ones in the polygon index vector given to the apply method.
    */
    inline const std::vector<int>& triangles()
        {
            return triangles_;
        }
};
}

#endif
