#include "MeshDecimate.h"
#include <crossforge/AssetIO/SAssetIO.h>
#include <chrono>
#include <iostream>
#include <unordered_map>
#include <functional>
#include <Prototypes/Actor/SLOD.h>
#include <igl/decimate.h>

using namespace Eigen;

template <typename Scalar, int Rows, int Cols>
struct std::hash<Eigen::Matrix<Scalar, Rows, Cols>> {
	// https://wjngkoh.wordpress.com/2015/03/04/c-hash-function-for-eigen-matrix-and-vector/
	size_t operator()(const Eigen::Matrix<Scalar, Rows, Cols>& matrix) const {
		size_t seed = 0;
		for (size_t i = 0; i < matrix.size(); ++i) {
			Scalar elem = *(matrix.data() + i);
			seed ^=
				std::hash<Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		}
		return seed;
	}
};

namespace CForge {

// TODO replace std::vector<std::vector<>> with std::vector<std::vector<>*>
// TODO inMesh zu const& machen // cleanup split funcs
bool MeshDecimator::decimateMesh(const CForge::T3DMesh<float>* inMesh, CForge::T3DMesh<float>* outMesh, float amount)
{
	printf("generating data structure\n");
	auto start = std::chrono::high_resolution_clock::now();
	
	uint32_t faceCount = 0;
	for (uint32_t i = 0; i < inMesh->submeshCount(); i++) {
		faceCount += inMesh->getSubmesh(i)->Faces.size();
	}
	
	Eigen::MatrixXd DV(inMesh->vertexCount(), 3);
	Eigen::MatrixXi DF(faceCount, 3);
	Eigen::VectorXi DuF; // used faces
	Eigen::VectorXi DuV; // used vertices
	
	for (uint32_t i = 0; i < inMesh->vertexCount(); i++) {
		DV.row(i) = inMesh->vertex(i).cast<double>();
	}
	
	std::vector<std::vector<uint32_t>> vertUsedInTri;
	vertUsedInTri.resize(DV.rows());
	
	std::vector<std::unordered_map<uint32_t,bool>> submeshTriangles; // contains Triangle IDs
	// add all faces from submeshes
	uint32_t startIndex = 0;
	for (uint32_t i = 0; i < inMesh->submeshCount(); i++) {
		const CForge::T3DMesh<float>::Submesh* submesh = inMesh->getSubmesh(i);
		std::unordered_map<uint32_t,bool> triIdx;
		
		auto faces = submesh->Faces;
		
		for (uint32_t j = 0; j < faces.size(); j++) {
			triIdx[startIndex + j] = true;
			Eigen::Vector3i subMVert(faces[j].Vertices[0], faces[j].Vertices[1], faces[j].Vertices[2]);
			for (uint8_t k = 0; k < 3; k++) {
				vertUsedInTri[faces[j].Vertices[k]].push_back(startIndex + j);
			}
			DF.row(startIndex + j) = subMVert;
		} // for every face of a submesh
		
		startIndex += submesh->Faces.size();
		submeshTriangles.push_back(triIdx);
	} // for all submeshes

	//std::cout << "DV\n" << DV << "\n";
	//std::cout << "DF\n" << DF << "\n";
	
	// datastructure for faster octree decimation
	std::vector<std::vector<uint32_t>*> DVnoMulUsedInTri;
	
	// merge points together
	std::vector<Eigen::Vector3d> DVnoMulVec;
	std::unordered_map<Eigen::Vector3d, uint32_t> DVnoMulVecMap;
	uint32_t index = 0;
	
	for (uint32_t i = 0; i < DV.rows(); i++) {
		
		Eigen::Vector3d vert = DV.row(i);
		int32_t idx;// = DVnoMulVec.size();
		auto itr = DVnoMulVecMap.find(vert);
		//auto itr = std::find(DVnoMulVec.begin(), DVnoMulVec.end(), vert);
		if (itr == DVnoMulVecMap.end()) { // vertex not contained, create new index
			DVnoMulVecMap[vert] = index;
			DVnoMulVec.push_back(vert);
			idx = index;//DVnoMulVec.size() - 1;
			
			std::vector<uint32_t>* newVec = new std::vector<uint32_t>(vertUsedInTri[i]);
			DVnoMulUsedInTri.push_back(newVec);
			index++;
		}
		else {
			idx = DVnoMulVecMap[vert]; //itr - DVnoMulVec.begin();
			int a = 1;
			std::vector<uint32_t>* vec = DVnoMulUsedInTri[idx];
			vec->insert(vec->end(), vertUsedInTri[i].begin(), vertUsedInTri[i].end());
		}
		
		// reindex face
		for (uint32_t triIdx : vertUsedInTri[i]) {
			for (uint32_t j = 0; j < 3; j++) {
				if (DF.row(triIdx)[j] == i)
					DF.row(triIdx)[j] = idx;
			}
		}
	} // for vertices
	
	Eigen::MatrixXd DVnoMul(DVnoMulVec.size(),3);
	for (uint32_t i = 0; i < DVnoMulVec.size(); i++) {
		DVnoMul.row(i) = DVnoMulVec[i];
	}
	
	//std::cout << "DVnoMul\n" << DVnoMul << "\n";
	//std::cout << "DV\n" << DV << "\n";
	//std::cout << "DF\n" << DF << "\n";
	//bool iglDecRes = igl::decimate(DVnoMul, DF, amount*DF.rows(), DVnoMul, DF, DuF, DuV);
	SLOD* pSLOD = SLOD::instance();
	bool iglDecRes = false;
	if (pSLOD->useLibigl)
		iglDecRes = igl::decimate(DVnoMul, DF, amount*DF.rows(), DVnoMul, DF, DuF, DuV);
	else
		iglDecRes = decimateOctree(DVnoMul, DF, &DuF, &DuV, amount*DF.rows(), DVnoMulUsedInTri);
	if (!iglDecRes) {
		std::cout << "an error occured while decimating!\n";
		//std::exit(-1);
		return false;
	}
	std::cout << "AFTERWARDS:\n";
	//std::cout << "DVnoMul\n" << DVnoMul << "\n";
	//std::cout << "DF\n" << DF << "\n";
	//std::cout << "DuF\n" << DuF << "\n";
	//std::cout << "DuV\n" << DuV << "\n";
			
	//std::cout << "startIndexes:\n";
	//for (uint32_t i : startIndexes)
	//	std::cout << i << ", ";
	//std::cout << "\nDstartIndexes:\n";
	//for (uint32_t i : DstartIndexes)
	//	std::cout << i << ", ";
	//std::cout << "\n";
	
	outMesh->clear();
	std::vector<Eigen::Vector3f> newVerts, newNormals, newTangents, newUVs, newColors;
	std::vector<uint32_t> UV;
	
	// copy all materials
	for (uint32_t i = 0; i < inMesh->materialCount(); i++) {
		T3DMesh<float>::Material* pM = new T3DMesh<float>::Material();
		pM->init( (*inMesh->getMaterial(i)) );
		outMesh->addMaterial(pM, false);
		outMesh->getMaterial(i)->Metallic = inMesh->getMaterial(i)->Metallic;
		outMesh->getMaterial(i)->Roughness = inMesh->getMaterial(i)->Roughness;
	}
	
	uint32_t oldStartSize = 0;
	uint32_t newStartSize = 0;
	// reassemble submeshes
	for (uint32_t i = 0; i < inMesh->submeshCount(); i++) {
		T3DMesh<float>::Submesh* pSubmesh = new T3DMesh<float>::Submesh();
		const T3DMesh<float>::Submesh* pOldSubmesh = inMesh->getSubmesh(i);
		
		std::vector<uint32_t> faces; // list of triangle IDs of Decimated Mesh corresponding to submesh
		uint32_t newTriAmount = 0;
		// only add used faces
		for (uint32_t k = 0; k < DuF.size(); k++) {
			
			auto itr = submeshTriangles[i].find(DuF[k]);
			if (itr != submeshTriangles[i].end()) {
				faces.push_back(DuF[k]);
				newTriAmount++;
			}
			
			//for (uint32_t j = 0; j < submeshTriangles[i].size(); j++) {
			//	uint32_t triID = submeshTriangles[i][j];
			//	if (DuF[k] == triID) {
			//		faces.push_back(triID);
			//		newTriAmount++;
			//		break;
			//	}
			//} // for old submesh tris
		} // for used tris
		
		std::vector<std::vector<Eigen::Vector4i>> newUVWs; //TODO rename newUVs conflict
		
		// add materials and set indices of submesh tris
		for (uint32_t j = 0; j < faces.size(); j++) {
			T3DMesh<float>::Face Face;
			const T3DMesh<float>::Face* pOldSMFace = &(pOldSubmesh->Faces[faces[j] - oldStartSize]);
			
			//Face.Material = pOldSMFace->Material;
		
			for (uint32_t k = 0; k < 3; k++) { //TODO fourth entry not used?
				int32_t vertID = DF.row(newStartSize + j)[k];
				newVerts.push_back(DVnoMul.row(vertID).cast<float>());
				uint32_t newSize = newVerts.size()-1;
				Face.Vertices[k] = newSize;
				
				////newNormals.push_back(Eigen::Vector3f::Zero());
				//newNormals.push_back(inMesh->normal(pOldSMFace->Normals[k]));
				//Face.Normals[k] = newSize;
				//
				//if (inMesh->textureCoordinatesCount() > 0) {
				//	newUVs.push_back(inMesh->textureCoordinate(pOldSMFace->UVWs[k]));
				//	Face.UVWs[k] = newSize;
				//}
				//if (inMesh->colorCount() > 0) {
				//	newColors.push_back(inMesh->color(pOldSMFace->Colors[k]));
				//	Face.Colors[k] = newSize;
				//}
				//if (inMesh->tangentCount() > 0) {
				//	if (pOldSMFace->Tangents[k] != -1) {
				//		newTangents.push_back(inMesh->tangent(pOldSMFace->Tangents[k]));
				//		Face.Tangents[k] = newSize;
				//	}
				//	else {
				//		newTangents.push_back(Vector3f::Zero());
				//		Face.Tangents[k] = newSize;
				//	}
				//}
			}//for[face indices]
			
			pSubmesh->Faces.push_back(Face);
		}//for[number of faces]
		outMesh->addSubmesh(pSubmesh, false);
		
		oldStartSize += inMesh->getSubmesh(i)->Faces.size();
		newStartSize += newTriAmount;
	} // for all submeshes

	outMesh->vertices(&newVerts);
	outMesh->normals(&newNormals);
	outMesh->tangents(&newTangents);
	outMesh->textureCoordinates(&newUVs);
	outMesh->colors(&newColors);
	
	for (uint32_t i = 0; i < DVnoMulUsedInTri.size(); i++) {
		delete DVnoMulUsedInTri[i];
	}
	
	//outMesh->computePerVertexNormals();
	
	long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() -start).count();
	std::cout << "Decimation Finished, time took: " << double(microseconds)*0.001 << "ms \n";
	return true;
} // decimateMesh func

bool MeshDecimator::insideAABB(Box BoundingBox, Eigen::Vector3f Vertex) {
	for (int i = 0; i < 3; i++) {
		if (Vertex[i] > BoundingBox.max()[i] || Vertex[i] < BoundingBox.min()[i])
			return false;
	}
	return true;
}//insideAABB

void MeshDecimator::releaseOctree(octreeNode* root) {
	for (uint8_t i = 0; i < 8; i++) {
		if (root->childs[i])
			releaseOctree(root->childs[i]);
	}
	delete root;
}

void MeshDecimator::createOctree(octreeNode* pNode, Eigen::MatrixXd* DV, std::vector<std::unordered_map<octreeNode*,bool>>* depthNodes) {
	// Proposed steps for creating the octree.
	// Step 1: Create all 8 AABBs of the respective octants
	Vector3f Maxs[8];
	Vector3f Mins[8];

	Vector3f Diag = pNode->BoundingBox.diagonal();
	Vector3f Min = pNode->BoundingBox.min();
	Vector3f Max = pNode->BoundingBox.max();
	Vector3f Center = Min + Diag / 2;

	Mins[0] = Min;
	Maxs[0] = Center;
	Mins[1] = Vector3f(Center.x(), Min.y(), Min.z()); // front bottom right
	Maxs[1] = Vector3f(Max.x(), Center.y(), Center.z());
	Mins[2] = Vector3f(Min.x(), Center.y(), Min.z()); // top left
	Maxs[2] = Vector3f(Center.x(), Max.y(), Center.z());
	Mins[3] = Vector3f(Center.x(), Center.y(), Min.z()); // top right
	Maxs[3] = Vector3f(Max.x(), Max.y(), Center.z());

	Mins[4] = Vector3f(Min.x(), Min.y(), Center.z()); // back bottom left
	Maxs[4] = Vector3f(Center.x(), Center.y(), Max.z());
	Mins[5] = Vector3f(Center.x(), Min.y(), Center.z()); // bottom right
	Maxs[5] = Vector3f(Max.x(), Center.y(), Max.z());
	Mins[6] = Vector3f(Min.x(), Center.y(), Center.z()); // top left
	Maxs[6] = Vector3f(Center.x(), Max.y(), Max.z());
	Mins[7] = Center;
	Maxs[7] = Max;

	// Step 2: create and initialize the 8 child nodes
	for (uint8_t i = 0; i < 8; ++i) {
		Box boundingBox;
		boundingBox.max(Maxs[i]);
		boundingBox.min(Mins[i]);

		pNode->childs[i] = new octreeNode();
		pNode->childs[i]->BoundingBox = boundingBox;
		pNode->childs[i]->parent = pNode;
		pNode->childs[i]->depth = pNode->depth + 1;
	}//for[each octant]

	// Step 3: Iterate other all the node's vertex IDs and sort them into the child nodes
	// Do not forget, that we have the insideAABB utility method
	for (auto id : pNode->VertexIDs) {
		// sort vertices into child nodes

		for (int i = 0; i < 8; i++) {
			if (insideAABB(pNode->childs[i]->BoundingBox, DV->row(id).cast<float>()))
				pNode->childs[i]->VertexIDs.push_back(id);
		}
	}//for[all vertexes]

	for (uint8_t i = 0; i < 8; ++i) {
		if (pNode->childs[i]->VertexIDs.size() != 0) {
			while (depthNodes->size() < pNode->childs[i]->depth)
				depthNodes->push_back(std::unordered_map<octreeNode*,bool>()); // TODO this does not look safe, pls change
			depthNodes->at(pNode->childs[i]->depth - 1)[pNode->childs[i]] = true;
		}
	}//for[octants]

	// Step4: Recursion
	for (uint8_t i = 0; i < 8; ++i) {
		// kill empty nodes
		if (pNode->childs[i]->VertexIDs.size() == 0) {
			delete pNode->childs[i];
			pNode->childs[i] = nullptr;
		}
		// call method again, if conditions for further subdivision are met
		else if (pNode->depth < m_MaxOctreeDepth && pNode->VertexIDs.size() > m_MaxLeafVertexCount) {
			//std::cout << "adding Node with depth " << pNode->Depth << " and size " << pNode->VertexIDs.size() << std::endl;
			createOctree(pNode->childs[i], DV, depthNodes);
		}
	}//for[octants]
}


bool MeshDecimator::decimateOctree(Eigen::MatrixXd& DV, Eigen::MatrixXi& DF, Eigen::VectorXi* DuF, Eigen::VectorXi* DuV, uint32_t faceAmount, std::vector<std::vector<uint32_t>*> DVnoMulUsedInTri) {
	if (DV.size() == 0 || DF.size() == 0)
		return false;

	uint32_t removeFaceCount = 0;
	std::vector<uint32_t> facesToRemove;
	std::unordered_map<uint32_t, bool> facesToRemoveMap;
	std::vector<uint32_t> pointsToRemove;
	std::vector<std::unordered_map<octreeNode*,bool>> depthNodes;

	printf("building Tree\n");
	// construct octree
	octreeNode* Root = new octreeNode();
	Root->depth = 0;
	Vector3f Min, Max;
	//set BB to first val or else we might have a prob
	for (uint8_t j = 0; j < 3; j++) {
		/*Root->BoundingBox.min()[j] = DV.row(0)[j];
		Root->BoundingBox.max()[j] = DV.row(0)[j];*/

		Min[j] = DV.row(0)[j];
		Max[j] = DV.row(0)[j];
	}
	for (uint32_t i = 0; i < DV.rows(); i++) {
		for (uint8_t j = 0; j < 3; j++) {
			float span = DV.row(i)[j];
			/*Root->BoundingBox.min()[j] = std::min(Root->BoundingBox.min()[j], span);
			Root->BoundingBox.max()[j] = std::max(Root->BoundingBox.max()[j], span);*/

			Min[j] = std::min(Min[j], span);
			Max[j] = std::max(Max[j], span);
		}
		Root->VertexIDs.push_back(i);
	}
	Root->BoundingBox.init(Min, Max);
	createOctree(Root, &DV, &depthNodes);

	// decimate, find targets
	float progressDiff = float(DF.rows()) - float(faceAmount);
	printf("progress:\n");
	while (DF.rows() - removeFaceCount > faceAmount) {
		printf("\r");
		printf("%.1f", removeFaceCount/progressDiff*100.0);
		
		if (depthNodes.size() == 0)
			break;
		// get last largest depth node
		std::unordered_map<octreeNode*,bool>* largestDepth = &(depthNodes.back());
		
		//auto itr = largestDepth->begin(); // the first element seems random enough? xd
		//std::advance(itr, rand()%largestDepth->size());
		octreeNode* parent = largestDepth->begin()->first->parent;
		
		//octreeNode* parent = largestDepth->at(rand()%largestDepth->size())->parent; // pick a random child to make decimation uniform
		if (!parent)
			break;
		//largestDepth->pop_back();

		// points which get merged into one
		std::vector<uint32_t> targetPoints;

		// join all child vertices into parent as one vertex
		for (uint32_t i = 0; i < 8; i++) {
			octreeNode* child = parent->childs[i];
			if (child) {
				for (uint32_t vertID : child->VertexIDs) {
					targetPoints.push_back(vertID);
				}
				// remove child pointer from depth list
				//largestDepth->erase(std::remove(largestDepth->begin(), largestDepth->end(), child), largestDepth->end());
				largestDepth->erase(child);

				delete child;
				parent->childs[i] = nullptr;
			}
		}

		pointsToRemove.insert(std::end(pointsToRemove), std::next(std::begin(targetPoints)), std::end(targetPoints));
		
		std::vector<uint32_t> addedFaces;
		joinPoints(&DV, &DF, targetPoints, DVnoMulUsedInTri, &addedFaces);
		for (uint32_t i = 0; i < addedFaces.size(); i++) {
			facesToRemoveMap[addedFaces[i]] = true;
		}
		removeFaceCount += addedFaces.size();
		
		// old concat
		//facesToRemove.insert(std::end(facesToRemove), std::begin(addedFaces), std::end(addedFaces));
		//removeFaceCount = facesToRemove.size();

		// push back new node
		parent->VertexIDs.push_back(targetPoints[0]);
		
		// reduce depth if no nodes are in it anymore
		if (depthNodes.back().empty())
			depthNodes.pop_back();
	}
	printf("\n");

	std::vector<uint32_t> DuFVec;
	// remove faces from DF
	for (uint32_t i = 0; i < DF.rows(); i++) {
		auto itr = facesToRemoveMap.find(i);
		if (itr == facesToRemoveMap.end())
			DuFVec.push_back(i);
		//if (std::find(facesToRemove.begin(), facesToRemove.end(), i) == facesToRemove.end())
		//	DuFVec.push_back(i);
	}
	Eigen::MatrixXi newDF = Eigen::MatrixXi(DuFVec.size(), 3);
	*DuF = Eigen::VectorXi(DuFVec.size());
	for (uint32_t i = 0; i < DuFVec.size(); i++) {
		//*DuF << DuFVec[i];
		(*DuF)[i] = DuFVec[i];
		newDF.row(i) = DF.row(DuFVec[i]);
	}
	//delete DF;
	DF = newDF;

	// TODO redo same with points // TODO check if necessary
	// free octree
	for (uint32_t i = 0; i < depthNodes.size(); i++) {
		for (auto& itr : depthNodes[i]) {
			delete itr.first;
		}
		
		//for (uint32_t j = 0; j < depthNodes[i].size(); j++) {
		//	delete depthNodes[i][j];
		//}
	}
	delete Root;
	//releaseOctree(Root);
	return true;
}

// merges points and returns vector of faces to remove
void MeshDecimator::joinPoints(Eigen::MatrixXd* DV, Eigen::MatrixXi* DF, const std::vector<uint32_t>& targets,
                               const std::vector<std::vector<uint32_t>*>& DVnoMulUsedInTri, std::vector<uint32_t>* removedFaces) {

	//std::vector<uint32_t> removedFaces;
	// calculate new Point position
	Eigen::Vector3f newPoint = Eigen::Vector3f::Zero();
	for (uint32_t i = 0; i < targets.size(); i++) {
		newPoint += DV->row(targets[i]).cast<float>();
	}
	newPoint /= targets.size();

	// set position of all points to new Point
	DV->row(targets[0]) = newPoint.cast<double>();
	//for (uint32_t i = 0; i < targets.size(); i++) {
	//	DV->row(targets[i]) = newPoint.cast<double>();
	//}
	
	
#if true // TODO causes minor holes
	for (uint32_t i = 0; i < targets.size(); i++) {
		std::vector<uint32_t>* triangles = DVnoMulUsedInTri[targets[i]];
		for (uint32_t j = 0; j < triangles->size(); j++) {
			for (uint8_t k = 0; k < 3; k++) {
				if (DF->row(triangles->at(j))[k] == targets[i])
					DF->row(triangles->at(j))[k] = targets[0];
			}
		}
	}
	// check which triangles got removed
	for (uint32_t i = 0; i < targets.size(); i++) {
		std::vector<uint32_t>* triangles = DVnoMulUsedInTri[targets[i]];
		for (uint32_t j = 0; j < triangles->size(); j++) {
			Eigen::Vector3i tri = DF->row(triangles->at(j));
			bool contained = tri[0] == tri[1] // if at least one vertex is doubled remove tri
			              || tri[1] == tri[2]
			              || tri[2] == tri[0];
			bool triple = tri[0] == tri[1] && tri[1] == tri[2];
			if (contained) {
				if (!triple) { // keep vertices outside targets in mind
					uint32_t index = tri[0] == tri[1] ? tri[2] : (tri[1] == tri[2] ? tri[0] : tri[1]);
					std::vector<uint32_t>* outsideVert = DVnoMulUsedInTri[index];
					if (outsideVert != triangles) {
						std::vector<uint32_t>::iterator it = std::find(outsideVert->begin(), outsideVert->end(), triangles->at(j));
						if(it != outsideVert->end())
							outsideVert->erase(it);
					}
				}
				if (std::find(removedFaces->begin(), removedFaces->end(), triangles->at(j)) == removedFaces->end())
					removedFaces->push_back(triangles->at(j));
				triangles->erase(triangles->begin()+j);
				j = 0;
			}
		}
	}
	for (uint32_t i = 1; i < targets.size(); i++) {
		std::vector<uint32_t>* triangles = DVnoMulUsedInTri[targets[i]];
		std::vector<uint32_t>* vec = DVnoMulUsedInTri[targets[0]];
		//for (uint32_t j = 0; j < triangles->size(); j++) {
		//	if (std::find(vec->begin(),vec->end(), triangles->at(j)) == vec->end()) {
		//		vec->push_back(triangles->at(j));
		//	}
		//}
		vec->insert(vec->end(), triangles->begin(), triangles->end());
		DVnoMulUsedInTri[targets[i]]->clear(); // empty
	}
#else
	// TODO make search faster by using bidirectional ref vert to tri
	for (uint32_t i = 0; i < DF->rows(); i++) {
		// check if tri contains more than 2 points in targets
		//Eigen::Vector3i tri = DF->row(i);

		uint8_t contained = 0;
		for (uint8_t j = 0; j < 3; j++) {
			if (std::find(targets.begin(), targets.end(), DF->row(i)[j]) != targets.end()) {
				// new point is in tri contained
				contained++;
				// point index to first target (new index of merged point)
				DF->row(i)[j] = targets[0];
			}
		}

		// 0. tri has 0 verts -> stays
		// 1. tri has 1 verts -> new point gets moved
		// 2. tri has 2 verts -> tri gets removed
		// 3. tri has 3 verts -> tri gets removed
		if (contained >= 2) { // remove tri
			removedFaces->push_back(i);
		}
	}
#endif
	//return removedFaces;
}

MeshDecimator::MeshDecimator(void) {
}//Constructor

MeshDecimator::~MeshDecimator(void) {
	//clear();
}//Destructor

}//CForge
