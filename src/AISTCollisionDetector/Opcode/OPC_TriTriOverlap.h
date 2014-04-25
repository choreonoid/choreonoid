
//! if OPC_TRITRI_EPSILON_TEST is true then we do a check (if |dv|<EPSILON then dv=0.0;) else no check is done (which is less robust, but faster)
#define LOCAL_EPSILON 0.000001f

//! sort so that a<=b
#define SORT(a,b)			\
	if(a>b)					\
	{						\
		const float c=a;	\
		a=b;				\
		b=c;				\
	}

//! Edge to edge test based on Franlin Antonio's gem: "Faster Line Segment Intersection", in Graphics Gems III, pp. 199-202
#define EDGE_EDGE_TEST(V0, U0, U1)						\
	Bx = U0[i0] - U1[i0];								\
	By = U0[i1] - U1[i1];								\
	Cx = V0[i0] - U0[i0];								\
	Cy = V0[i1] - U0[i1];								\
	f  = Ay*Bx - Ax*By;									\
	d  = By*Cx - Bx*Cy;									\
	if((f>0.0f && d>=0.0f && d<=f) || (f<0.0f && d<=0.0f && d>=f))	\
	{													\
		const float e=Ax*Cy - Ay*Cx;					\
		if(f>0.0f)										\
		{												\
			if(e>=0.0f && e<=f) return TRUE;			\
		}												\
		else											\
		{												\
			if(e<=0.0f && e>=f) return TRUE;			\
		}												\
	}

//! TO BE DOCUMENTED
#define EDGE_AGAINST_TRI_EDGES(V0, V1, U0, U1, U2)		\
{														\
	float Bx,By,Cx,Cy,d,f;								\
	const float Ax = V1[i0] - V0[i0];					\
	const float Ay = V1[i1] - V0[i1];					\
	/* test edge U0,U1 against V0,V1 */					\
	EDGE_EDGE_TEST(V0, U0, U1);							\
	/* test edge U1,U2 against V0,V1 */					\
	EDGE_EDGE_TEST(V0, U1, U2);							\
	/* test edge U2,U1 against V0,V1 */					\
	EDGE_EDGE_TEST(V0, U2, U0);							\
}

//! TO BE DOCUMENTED
#define POINT_IN_TRI(V0, U0, U1, U2)					\
{														\
	/* is T1 completly inside T2? */					\
	/* check if V0 is inside tri(U0,U1,U2) */			\
	float a  = U1[i1] - U0[i1];							\
	float b  = -(U1[i0] - U0[i0]);						\
	float c  = -a*U0[i0] - b*U0[i1];					\
	float d0 = a*V0[i0] + b*V0[i1] + c;					\
														\
	a  = U2[i1] - U1[i1];								\
	b  = -(U2[i0] - U1[i0]);							\
	c  = -a*U1[i0] - b*U1[i1];							\
	const float d1 = a*V0[i0] + b*V0[i1] + c;			\
														\
	a  = U0[i1] - U2[i1];								\
	b  = -(U0[i0] - U2[i0]);							\
	c  = -a*U2[i0] - b*U2[i1];							\
	const float d2 = a*V0[i0] + b*V0[i1] + c;			\
	if(d0*d1>0.0f)										\
	{													\
		if(d0*d2>0.0f) return TRUE;						\
	}													\
}

//! TO BE DOCUMENTED
BOOL CoplanarTriTri(const Point& n, const Point& v0, const Point& v1, const Point& v2, const Point& u0, const Point& u1, const Point& u2)
{
	float A[3];
	short i0,i1;
	/* first project onto an axis-aligned plane, that maximizes the area */
	/* of the triangles, compute indices: i0,i1. */
	A[0] = fabsf(n[0]);
	A[1] = fabsf(n[1]);
	A[2] = fabsf(n[2]);
	if(A[0]>A[1])
	{
		if(A[0]>A[2])
		{
			i0=1;      /* A[0] is greatest */
			i1=2;
		}
		else
		{
			i0=0;      /* A[2] is greatest */
			i1=1;
		}
	}
	else   /* A[0]<=A[1] */
	{
		if(A[2]>A[1])
		{
			i0=0;      /* A[2] is greatest */
			i1=1;
		}
		else
		{
			i0=0;      /* A[1] is greatest */
			i1=2;
		}
	}

	/* test all edges of triangle 1 against the edges of triangle 2 */
	EDGE_AGAINST_TRI_EDGES(v0, v1, u0, u1, u2);
	EDGE_AGAINST_TRI_EDGES(v1, v2, u0, u1, u2);
	EDGE_AGAINST_TRI_EDGES(v2, v0, u0, u1, u2);

	/* finally, test if tri1 is totally contained in tri2 or vice versa */
	POINT_IN_TRI(v0, u0, u1, u2);
	POINT_IN_TRI(u0, v0, v1, v2);

	return FALSE;
}

//! TO BE DOCUMENTED
#define NEWCOMPUTE_INTERVALS(VV0, VV1, VV2, D0, D1, D2, D0D1, D0D2, A, B, C, X0, X1)	\
{																						\
	if(D0D1>0.0f)																		\
	{																					\
		/* here we know that D0D2<=0.0 */												\
		/* that is D0, D1 are on the same side, D2 on the other or on the plane */		\
		A=VV2; B=(VV0 - VV2)*D2; C=(VV1 - VV2)*D2; X0=D2 - D0; X1=D2 - D1;				\
	}																					\
	else if(D0D2>0.0f)																	\
	{																					\
		/* here we know that d0d1<=0.0 */												\
		A=VV1; B=(VV0 - VV1)*D1; C=(VV2 - VV1)*D1; X0=D1 - D0; X1=D1 - D2;				\
	}																					\
	else if(D1*D2>0.0f || D0!=0.0f)														\
	{																					\
		/* here we know that d0d1<=0.0 or that D0!=0.0 */								\
		A=VV0; B=(VV1 - VV0)*D0; C=(VV2 - VV0)*D0; X0=D0 - D1; X1=D0 - D2;				\
	}																					\
	else if(D1!=0.0f)																	\
	{																					\
		A=VV1; B=(VV0 - VV1)*D1; C=(VV2 - VV1)*D1; X0=D1 - D0; X1=D1 - D2;				\
	}																					\
	else if(D2!=0.0f)																	\
	{																					\
		A=VV2; B=(VV0 - VV2)*D2; C=(VV1 - VV2)*D2; X0=D2 - D0; X1=D2 - D1;				\
	}																					\
	else																				\
	{																					\
		/* triangles are coplanar */													\
		return CoplanarTriTri(N1, V0, V1, V2, U0, U1, U2);								\
	}																					\
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Triangle/triangle intersection test routine,
 *	by Tomas Moller, 1997.
 *	See article "A Fast Triangle-Triangle Intersection Test",
 *	Journal of Graphics Tools, 2(2), 1997
 *
 *	Updated June 1999: removed the divisions -- a little faster now!
 *	Updated October 1999: added {} to CROSS and SUB macros 
 *
 *	int NoDivTriTriIsect(float V0[3],float V1[3],float V2[3],
 *                      float U0[3],float U1[3],float U2[3])
 *
 *	\param		V0		[in] triangle 0, vertex 0
 *	\param		V1		[in] triangle 0, vertex 1
 *	\param		V2		[in] triangle 0, vertex 2
 *	\param		U0		[in] triangle 1, vertex 0
 *	\param		U1		[in] triangle 1, vertex 1
 *	\param		U2		[in] triangle 1, vertex 2
 *	\return		true if triangles overlap
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
BOOL AABBTreeCollider::TriTriOverlap(const Point& V0, const Point& V1, const Point& V2, const Point& U0, const Point& U1, const Point& U2)
{

	// Stats
	mNbPrimPrimTests++;

	// Modified by S-cubed, Inc.
	// Detect collisions by using TriOverlap.cpp instead of the collision detection by OPCODE

        cnoid::collision_data c_pair;
        cnoid::Vector3 i0, i1, i2;
	cnoid::Vector3 p0, p1, p2;

	// Convert coordinates to match the interface of tri_tri_overlap()@TriOerlap.cpp
	// Ice/IcePointer => Vector3

	i0[0] = V0.x;        i0[1] = V0.y;        i0[2] = V0.z;
	i1[0] = V1.x;        i1[1] = V1.y;        i1[2] = V1.z;
	i2[0] = V2.x;        i2[1] = V2.y;        i2[2] = V2.z;

	p0[0] = U0.x;	     p0[1] = U0.y;        p0[2] = U0.z;
	p1[0] = U1.x;	     p1[1] = U1.y;        p1[2] = U1.z;
	p2[0] = U2.x;	     p2[1] = U2.y;        p2[2] = U2.z;
	
	if(collisionPairInserter &&
           collisionPairInserter->detectTriTriOverlap(i0, i1, i2, p0, p1, p2, &c_pair)){
            /*
		insert_collision_pair(mNowNode0, mNowNode1, mId0, mId1,
				      c_pair.num_of_i_points,
				      c_pair.i_points,
				      c_pair.n_vector,
				      c_pair.depth,
				      c_pair.n,
				      c_pair.m,
				      c_pair.c_type,
				      (MeshInterface*)mIMesh0,
				      (MeshInterface*)mIMesh1);
            */
            collisionPairInserter->apply(mNowNode0, mNowNode1, mId0, mId1,
                                         c_pair.num_of_i_points,
                                         c_pair.i_points,
                                         c_pair.n_vector,
                                         c_pair.depth,
                                         c_pair.n,
                                         c_pair.m,
                                         c_pair.c_type,
                                         (MeshInterface*)mIMesh0,
                                         (MeshInterface*)mIMesh1);
            return TRUE;
	}
	return FALSE;
}
