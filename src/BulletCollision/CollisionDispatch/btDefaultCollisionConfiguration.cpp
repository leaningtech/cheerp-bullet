/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btDefaultCollisionConfiguration.h"

#include "BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM
#include "BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.h"
#endif //USE_BUGGY_SPHERE_BOX_ALGORITHM
#include "BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"



#include "LinearMath/btStackAlloc.h"
#include "LinearMath/btPoolAllocator.h"





btDefaultCollisionConfiguration::btDefaultCollisionConfiguration(const btDefaultCollisionConstructionInfo& constructionInfo)
//btDefaultCollisionConfiguration::btDefaultCollisionConfiguration(btStackAlloc*	stackAlloc,btPoolAllocator*	persistentManifoldPool,btPoolAllocator*	collisionAlgorithmPool)
{

	m_simplexSolver = new btVoronoiSimplexSolver();

	if (constructionInfo.m_useEpaPenetrationAlgorithm)
	{
		m_pdSolver = new btGjkEpaPenetrationDepthSolver;
	}else
	{
		m_pdSolver = new btMinkowskiPenetrationDepthSolver;
	}
	
	//default CreationFunctions, filling the m_doubleDispatch table
	m_convexConvexCreateFunc = new btConvexConvexAlgorithm::CreateFunc(m_simplexSolver,m_pdSolver);
	m_convexConcaveCreateFunc = new btConvexConcaveCollisionAlgorithm::CreateFunc;
	m_swappedConvexConcaveCreateFunc = new btConvexConcaveCollisionAlgorithm::SwappedCreateFunc;
	m_compoundCreateFunc = new btCompoundCollisionAlgorithm::CreateFunc;
	m_swappedCompoundCreateFunc = new btCompoundCollisionAlgorithm::SwappedCreateFunc;
	m_emptyCreateFunc = new btEmptyAlgorithm::CreateFunc;
	
	m_sphereSphereCF = new btSphereSphereCollisionAlgorithm::CreateFunc;
#ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM
	m_sphereBoxCF = new btSphereBoxCollisionAlgorithm::CreateFunc;
	m_boxSphereCF = new btSphereBoxCollisionAlgorithm::CreateFunc;
	m_boxSphereCF->m_swapped = true;
#endif //USE_BUGGY_SPHERE_BOX_ALGORITHM

	m_sphereTriangleCF = new btSphereTriangleCollisionAlgorithm::CreateFunc;
	m_triangleSphereCF = new btSphereTriangleCollisionAlgorithm::CreateFunc;
	m_triangleSphereCF->m_swapped = true;
	
	m_boxBoxCF = new btBoxBoxCollisionAlgorithm::CreateFunc;

	//convex versus plane
	m_convexPlaneCF = new btConvexPlaneCollisionAlgorithm::CreateFunc;
	m_planeConvexCF = new btConvexPlaneCollisionAlgorithm::CreateFunc;
	m_planeConvexCF->m_swapped = true;
	
	///calculate maximum element size, big enough to fit any collision algorithm in the memory pool
	int maxSize = sizeof(btConvexConvexAlgorithm);
	int maxSize2 = sizeof(btConvexConcaveCollisionAlgorithm);
	int maxSize3 = sizeof(btCompoundCollisionAlgorithm);
	int sl = sizeof(btConvexSeparatingDistanceUtil);
	sl = sizeof(btGjkPairDetector);
	int	collisionAlgorithmMaxElementSize = btMax(maxSize,constructionInfo.m_customCollisionAlgorithmMaxElementSize);
	collisionAlgorithmMaxElementSize = btMax(collisionAlgorithmMaxElementSize,maxSize2);
	collisionAlgorithmMaxElementSize = btMax(collisionAlgorithmMaxElementSize,maxSize3);

	if (constructionInfo.m_stackAlloc)
	{
		m_ownsStackAllocator = false;
		this->m_stackAlloc = constructionInfo.m_stackAlloc;
	} else
	{
		m_ownsStackAllocator = true;
		m_stackAlloc = new btStackAlloc(constructionInfo.m_defaultStackAllocatorSize);
	}
}

btDefaultCollisionConfiguration::~btDefaultCollisionConfiguration()
{
	if (m_ownsStackAllocator)
	{
		m_stackAlloc->destroy();
		delete m_stackAlloc;
	}

	delete 	m_convexConvexCreateFunc;

	delete  m_convexConcaveCreateFunc;
	delete  m_swappedConvexConcaveCreateFunc;

	delete  m_compoundCreateFunc;

	delete  m_swappedCompoundCreateFunc;

	delete  m_emptyCreateFunc;

	delete  m_sphereSphereCF;

#ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM
	delete  m_sphereBoxCF;
	delete  m_boxSphereCF;
#endif //USE_BUGGY_SPHERE_BOX_ALGORITHM

	delete  m_sphereTriangleCF;
	delete  m_triangleSphereCF;
	delete  m_boxBoxCF;

	delete  m_convexPlaneCF;
	delete  m_planeConvexCF;

	delete m_simplexSolver;
	
	delete m_pdSolver;


}


btCollisionAlgorithmCreateFunc* btDefaultCollisionConfiguration::getCollisionAlgorithmCreateFunc(int proxyType0,int proxyType1)
{



	if ((proxyType0 == SPHERE_SHAPE_PROXYTYPE) && (proxyType1==SPHERE_SHAPE_PROXYTYPE))
	{
		return	m_sphereSphereCF;
	}
#ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM
	if ((proxyType0 == SPHERE_SHAPE_PROXYTYPE) && (proxyType1==BOX_SHAPE_PROXYTYPE))
	{
		return	m_sphereBoxCF;
	}

	if ((proxyType0 == BOX_SHAPE_PROXYTYPE ) && (proxyType1==SPHERE_SHAPE_PROXYTYPE))
	{
		return	m_boxSphereCF;
	}
#endif //USE_BUGGY_SPHERE_BOX_ALGORITHM


	if ((proxyType0 == SPHERE_SHAPE_PROXYTYPE ) && (proxyType1==TRIANGLE_SHAPE_PROXYTYPE))
	{
		return	m_sphereTriangleCF;
	}

	if ((proxyType0 == TRIANGLE_SHAPE_PROXYTYPE  ) && (proxyType1==SPHERE_SHAPE_PROXYTYPE))
	{
		return	m_triangleSphereCF;
	} 

	if ((proxyType0 == BOX_SHAPE_PROXYTYPE) && (proxyType1 == BOX_SHAPE_PROXYTYPE))
	{
		return m_boxBoxCF;
	}
	
	if (btBroadphaseProxy::isConvex(proxyType0) && (proxyType1 == STATIC_PLANE_PROXYTYPE))
	{
		return m_convexPlaneCF;
	}

	if (btBroadphaseProxy::isConvex(proxyType1) && (proxyType0 == STATIC_PLANE_PROXYTYPE))
	{
		return m_planeConvexCF;
	}
	


	if (btBroadphaseProxy::isConvex(proxyType0) && btBroadphaseProxy::isConvex(proxyType1))
	{
		return m_convexConvexCreateFunc;
	}

	if (btBroadphaseProxy::isConvex(proxyType0) && btBroadphaseProxy::isConcave(proxyType1))
	{
		return m_convexConcaveCreateFunc;
	}

	if (btBroadphaseProxy::isConvex(proxyType1) && btBroadphaseProxy::isConcave(proxyType0))
	{
		return m_swappedConvexConcaveCreateFunc;
	}

	if (btBroadphaseProxy::isCompound(proxyType0))
	{
		return m_compoundCreateFunc;
	} else
	{
		if (btBroadphaseProxy::isCompound(proxyType1))
		{
			return m_swappedCompoundCreateFunc;
		}
	}

	//failed to find an algorithm
	return m_emptyCreateFunc;
}

void btDefaultCollisionConfiguration::setConvexConvexMultipointIterations(int numPerturbationIterations, int minimumPointsPerturbationThreshold)
{
	btConvexConvexAlgorithm::CreateFunc* convexConvex = (btConvexConvexAlgorithm::CreateFunc*) m_convexConvexCreateFunc;
	convexConvex->m_numPerturbationIterations = numPerturbationIterations;
	convexConvex->m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;
}

void	btDefaultCollisionConfiguration::setPlaneConvexMultipointIterations(int numPerturbationIterations, int minimumPointsPerturbationThreshold)
{
	btConvexPlaneCollisionAlgorithm::CreateFunc* cpCF = (btConvexPlaneCollisionAlgorithm::CreateFunc*)m_convexPlaneCF;
	cpCF->m_numPerturbationIterations = numPerturbationIterations;
	cpCF->m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;
	
	btConvexPlaneCollisionAlgorithm::CreateFunc* pcCF = (btConvexPlaneCollisionAlgorithm::CreateFunc*)m_planeConvexCF;
	pcCF->m_numPerturbationIterations = numPerturbationIterations;
	pcCF->m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;
}
