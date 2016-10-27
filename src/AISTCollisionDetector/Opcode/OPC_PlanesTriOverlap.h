#ifdef for
#undef for
#endif

#include "../CollisionData.h"
#include "../CollisionPairInserter.h"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Planes-triangle overlap test.
 *	\param		in_clip_mask	[in] bitmask for active planes
 *	\return		TRUE if triangle overlap planes
 *	\warning	THIS IS A CONSERVATIVE TEST !! Some triangles will be returned as intersecting, while they're not!
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline_ BOOL PlanesCollider::PlanesTriOverlap(udword in_clip_mask)
{
	// Stats
	mNbVolumePrimTests++;

	const Plane* p = mPlanes;
	udword Mask = 1;

	while(Mask<=in_clip_mask)
	{
		if(in_clip_mask & Mask)
		{
			float d0 = p->Distance(*mVP.Vertex[0]);
			float d1 = p->Distance(*mVP.Vertex[1]);
			float d2 = p->Distance(*mVP.Vertex[2]);
			if(d0>0.0f && d1>0.0f && d2>0.0f)	return FALSE;
//			if(!(IR(d0)&SIGN_BITMASK) && !(IR(d1)&SIGN_BITMASK) && !(IR(d2)&SIGN_BITMASK))	return FALSE;
                        if (collisionPairInserter){
                            CollisionPairInserter &c = *collisionPairInserter;
                            cnoid::collision_data cdata;
                            cdata.num_of_i_points = 2;
                            cdata.i_point_new[0] = cdata.i_point_new[1] = 1;
                            cdata.i_point_new[2] = cdata.i_point_new[3] = 0;
                            cdata.n_vector = c.CD_Rot1 * cnoid::Vector3(p->n.x, p->n.y, p->n.z);
                            Point p1, p2;
                            if (d0<=0 && d1>0 && d2>0){
                                cdata.depth = -d0;
                                p1 = ((*mVP.Vertex[0])*d1-(*mVP.Vertex[1])*d0)/(d1-d0);
                                p2 = ((*mVP.Vertex[0])*d2-(*mVP.Vertex[2])*d0)/(d2-d0);
                            }else if(d0>0 && d1<=0 && d2>0){
                                cdata.depth = -d1;
                                p1 = ((*mVP.Vertex[1])*d0-(*mVP.Vertex[0])*d1)/(d0-d1);
                                p2 = ((*mVP.Vertex[1])*d2-(*mVP.Vertex[2])*d1)/(d2-d1);
                            }else if(d0>0 && d1>0 && d2<=0){
                                cdata.depth = -d2;
                                p1 = ((*mVP.Vertex[2])*d0-(*mVP.Vertex[0])*d2)/(d0-d2);
                                p2 = ((*mVP.Vertex[2])*d1-(*mVP.Vertex[1])*d2)/(d1-d2);
                            }else if(d0<=0 && d1<=0 && d2>0){
                                cdata.depth = d0 > d1 ? -d1 : -d0;
                                p1 = ((*mVP.Vertex[0])*d2-(*mVP.Vertex[2])*d0)/(d2-d0);
                                p2 = ((*mVP.Vertex[1])*d2-(*mVP.Vertex[2])*d1)/(d2-d1);
                            }else if(d0<=0 && d1>0 && d2<=0){
                                cdata.depth = d0 > d2 ? -d2 : -d0;
                                p1 = ((*mVP.Vertex[0])*d1-(*mVP.Vertex[1])*d0)/(d1-d0);
                                p2 = ((*mVP.Vertex[2])*d1-(*mVP.Vertex[1])*d2)/(d1-d2);
                            }else if(d0>0 && d1<=0 && d2<=0){
                                cdata.depth = d1 > d2 ? -d2 : -d1;
                                p1 = ((*mVP.Vertex[1])*d0-(*mVP.Vertex[0])*d1)/(d0-d1);
                                p2 = ((*mVP.Vertex[2])*d0-(*mVP.Vertex[0])*d2)/(d0-d2);
                            }
                            if (d0>0||d1>0||d2>0){
                                //cnoid::Vector3 v1, v2;
                                //cnoid::getVector3(v1, p1);
                                //cnoid::getVector3(v2, p2);
                                
                                cnoid::Vector3 v2(p2.x, p2.y, p2.z);
                                
                                cdata.i_points[0] = c.CD_s1 * (c.CD_Rot1 * cnoid::Vector3(p1.x, p1.y, p1.z) + c.CD_Trans1);
                                cdata.i_points[1] = c.CD_s1 * (c.CD_Rot1 * cnoid::Vector3(p2.x, p2.y, p2.z) + c.CD_Trans1);
                                collisionPairInserter->collisions().push_back(cdata);
                            }
                        }
		}
		Mask+=Mask;
		p++;
	}
/*
	for(udword i=0;i<6;i++)
	{
		float d0 = p[i].Distance(mLeafVerts[0]);
		float d1 = p[i].Distance(mLeafVerts[1]);
		float d2 = p[i].Distance(mLeafVerts[2]);
		if(d0>0.0f && d1>0.0f && d2>0.0f)	return false;
	}
*/
	return TRUE;
}
