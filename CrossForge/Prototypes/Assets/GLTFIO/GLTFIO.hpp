/*****************************************************************************\
*                                                                           *
* File(s): GLTFIO.hpp and GLTFIO.cpp                                            *
*                                                                           *
* Content: Import/Export class for glTF format using tinygltf   *
*                                                   *
*                                                                           *
*                                                                           *
* Author(s): Tom Uhlmann                                                    *
*                                                                           *
*                                                                           *
* The file(s) mentioned above are provided as is under the terms of the     *
* MIT License without any warranty or guaranty to work properly.            *
* For additional license, copyright and contact/support issues see the      *
* supplied documentation.                                                   *
*                                                                           *
\****************************************************************************/
#pragma once

#include <crossforge/AssetIO/I3DMeshIO.h>
#include <tiny_gltf.h>
#include <algorithm>
#include <iostream>

#include "GLTFIOutil.hpp"

//TODO documentation

namespace CForge {

/**
 * \class GLTFIO
 * \brief Export and Import gltf files.
*/
class GLTFIO : public GLTFIOutil {
public:
	static GLTFIO& instance() {
		static GLTFIO instance;
		return instance;
	}
	static void load(const std::string Filepath, T3DMesh<float>* pMesh) {
		instance().loadIntern(Filepath,pMesh);
		instance().clearInternal();
	}
	static void store(const std::string Filepath, const T3DMesh<float>* pMesh) {
		instance().storeIntern(Filepath,pMesh);
		instance().clearInternal();
	}

	//TODOf(skade) for SAssetIO, but probably problem with this beeing singleton?
	//void release(void);
	static bool accepted(const std::string Filepath, I3DMeshIO::Operation Op);

private:
	void clearInternal() {
		m_filePath = "";

		m_model = tinygltf::Model();
		m_pMesh = nullptr;
		m_pCMesh = nullptr; //TODO(skade)
		// Data for every primitive is stored in a separate vector.

		m_coord.clear();
		m_normal.clear();
		m_tangent.clear();
		m_texCoord.clear();
		m_color.clear();
		m_joint.clear();
		m_weight.clear();

		m_primitiveIndexRanges.clear();
		m_offsets.clear(); //TODO(skade) type
		m_materialIndex = 0; //TODO(skade) type
	};
	void loadIntern(const std::string Filepath, T3DMesh<float>* pMesh);
	void storeIntern(const std::string Filepath, const T3DMesh<float>* pMesh);

	//TODO(skade) remove singleton status, keep load store static and remove members
	std::string m_filePath = "";

	tinygltf::Model m_model;
	T3DMesh<float>* m_pMesh = nullptr;
	T3DMesh<float>* m_pCMesh = nullptr; //TODO(skade)
	// Data for every primitive is stored in a separate vector.

	std::vector<Eigen::Matrix<float, 3, 1>> m_coord;
	std::vector<Eigen::Matrix<float, 3, 1>> m_normal;
	std::vector<Eigen::Matrix<float, 3, 1>> m_tangent;
	std::vector<Eigen::Matrix<float, 2, 1>> m_texCoord;
	std::vector<Eigen::Matrix<float, 4, 1>> m_color;
	std::vector<Eigen::Matrix<float, 4, 1>> m_joint;
	std::vector<Eigen::Matrix<float, 4, 1>> m_weight;

	std::vector<std::pair<int32_t, int32_t>> m_primitiveIndexRanges;
	std::vector<unsigned long> m_offsets; //TODO(skade) type
	unsigned long m_materialIndex = 0; //TODO(skade) type


	std::vector<int> getMeshIndexByCrossForgeVertexIndex(int index);

	int getNodeIndexByName(const std::string& name);

	GLTFIO() {};
public:
	GLTFIO(GLTFIO const&) = delete;
	void operator=(GLTFIO const&) = delete;

private:
//TODO(skade) move elsewhere
#pragma region accessor_read
	template<class T>
	void readBuffer(unsigned char* pBuffer, const int element_count, const int offset, const int component_count,
	                const bool is_matrix, const int stride, std::vector<T>* pData) {
		int type_size = sizeof(T);

		if (is_matrix) {
			assert(component_count == 16);
			assert(type_size == 4);
		}

		pData->reserve(element_count * component_count);

		T* raw_data = (T*)(pBuffer + offset);

		int jump = stride / type_size;
		if (jump == 0) jump = component_count;

		for (int i = 0; i < element_count; i++) {
			int index = i * jump;
			for (int k = 0; k < component_count; k++) {
				T tmp = raw_data[index + k];
				pData->push_back(tmp);
			}
		}
	}

	template<class T>
	void getAccessorDataScalar(const int accessor, std::vector<T>* pData) {
		tinygltf::Accessor acc = m_model.accessors[accessor];
		tinygltf::BufferView buffView = m_model.bufferViews[acc.bufferView];
		tinygltf::Buffer buff = m_model.buffers[buffView.buffer];

		if (acc.type != TINYGLTF_TYPE_SCALAR) {
			//std::cout << "Called getAccessorDataScalar on a non scalar accessor!" << std::endl;
			return;
		}

		if (acc.componentType == TINYGLTF_COMPONENT_TYPE_BYTE) {
			std::vector<char> data;

			readBuffer(buff.data.data(), acc.count, acc.byteOffset + buffView.byteOffset, 1, false, 0, &data);
			for (auto d : data) pData->push_back((T)d);
			return;
		}

		if (acc.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE) {
			std::vector<unsigned char> data;

			readBuffer(buff.data.data(), acc.count, acc.byteOffset + buffView.byteOffset, 1, false, 0, &data);
			for (auto d : data) pData->push_back((T)d);
			return;
		}

		if (acc.componentType == TINYGLTF_COMPONENT_TYPE_SHORT) {
			std::vector<short> data;

			readBuffer(buff.data.data(), acc.count, acc.byteOffset + buffView.byteOffset, 1, false, 0, &data);
			for (auto d : data) pData->push_back((T)d);
			return;
		}

		if (acc.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) {
			std::vector<unsigned short> data;

			readBuffer(buff.data.data(), acc.count, acc.byteOffset + buffView.byteOffset, 1, false, 0, &data);
			for (auto d : data) pData->push_back((T)d);
			return;
		}

		if (acc.componentType == TINYGLTF_COMPONENT_TYPE_INT) {
			std::vector<int32_t> data;

			readBuffer(buff.data.data(), acc.count, acc.byteOffset + buffView.byteOffset, 1, false, 0, &data);
			for (auto d : data) pData->push_back((T)d);
			return;
		}

		if (acc.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT) {
			std::vector<int32_t> data;

			readBuffer(buff.data.data(), acc.count, acc.byteOffset + buffView.byteOffset, 1, false, 0, &data);
			for (auto d : data) pData->push_back((T)d);
			return;
		}

		if (acc.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT) {
			std::vector<float> data;

			readBuffer(buff.data.data(), acc.count, acc.byteOffset + buffView.byteOffset, 1, false, 0, &data);
			for (auto d : data) pData->push_back((T)d);
			return;
		}
	}

	void getAccessorDataScalarFloat(const int accessor, std::vector<float>* pData);

	template<class T>
	void getAccessorData(const int accessor, std::vector<std::vector<T>>* pData) {
		tinygltf::Accessor acc = m_model.accessors[accessor];
		tinygltf::BufferView buffView = m_model.bufferViews[acc.bufferView];
		tinygltf::Buffer buff = m_model.buffers[buffView.buffer];

		if (acc.type == TINYGLTF_TYPE_SCALAR) {
			//std::cout << "Called getAccessorData on a scalar accessor!" << std::endl;
			return;
		}

		int nComponents = componentCount(acc.type);

		std::vector<T> unstructured;

		int remaining_size = buff.data.size() - acc.byteOffset - buffView.byteOffset;

		readBuffer(buff.data.data(), acc.count, acc.byteOffset + buffView.byteOffset, nComponents, componentIsMatrix(acc.type), buffView.byteStride, &unstructured);

		for (int i = 0; i < unstructured.size(); i += nComponents) {
			std::vector<T> toAdd;
			for (int j = 0; j < nComponents; j++) {
				toAdd.push_back(unstructured[i + j]);
			}
			pData->push_back(toAdd);
		}
	}

	int32_t getSparseAccessorData(const int accessor, std::vector<int32_t>* pIndices, std::vector<std::vector<float>>* pData);

	void getAccessorDataFloat(const int accessor, std::vector<std::vector<float>>* pData);

	void getAccessorData(const int accessor, std::vector<Eigen::Vector3f>* pData);

	void getAccessorData(const int accessor, std::vector<Eigen::Vector4f>* pData);

	void getAccessorData(const int accessor, std::vector<Eigen::Quaternionf>* pData);
#pragma endregion

#pragma region read
	void readMeshes();

	T3DMesh<float>::Submesh* readPrimitive(tinygltf::Primitive* pPrimitive);

	void readAttributes(tinygltf::Primitive* pPrimitive);

	T3DMesh<float>::Submesh* readSubMeshes(tinygltf::Primitive* pPrimitive);

	void readFaces(tinygltf::Primitive* pPrimitive, std::vector<T3DMesh<float>::Face>* faces);

	void readMaterial(const int m_materialIndex, T3DMesh<float>::Material* pMaterial);

	void readNodes();

	std::string getTexturePath(const int textureIndex);

	void readSkeletalAnimations();

	void readSkinningData();

	void readMorphTargets();
#pragma endregion

#pragma region accessor_write
	template<class T>
	void writeBuffer(std::vector<unsigned char>* pBuffer, const int offset, const int component_count, const bool is_matrix, const int stride, const std::vector<T>* pData) {
		int type_size = sizeof(T);

		if (is_matrix) {
			assert(component_count == 16);
			assert(type_size == 4);
		}

		while (pBuffer->size() < offset) pBuffer->push_back(0);

		int index = offset;

		for (int i = 0; i < pData->size(); i++) {
			T element = (*pData)[i];
			unsigned char* as_char_pointer = (unsigned char*)&element;

			for (int k = 0; k < type_size; k++) {
				if (index == pBuffer->size()) pBuffer->push_back(as_char_pointer[k]);
				else (*pBuffer)[index] = as_char_pointer[k];
				index++;
			}

			for (int k = 0; k < stride; k++) {
				if (index == pBuffer->size()) pBuffer->push_back(0);
				index++;
			}
		}
	}

	template<class T>
	void writeAccessorDataScalar(const int bufferIndex, std::vector<T>* pData) {
		std::cout << "write accessor size: " << pData->size() << ", scalar" << std::endl;

		tinygltf::Buffer* pBuffer = &m_model.buffers[bufferIndex];

		tinygltf::Accessor accessor;

		T min = std::numeric_limits<T>::max();
		T max = std::numeric_limits<T>::lowest();

		for (int i = 0; i < pData->size(); i++) {
			T element = (*pData)[i];
			if (element < min) min = element;
			if (element > max) max = element;
		}

		accessor.minValues.push_back(min);
		accessor.maxValues.push_back(max);

		T t = 0;

		accessor.bufferView = m_model.bufferViews.size();
		accessor.byteOffset = (size_t)0;
		accessor.componentType = getGltfComponentType(t);
		accessor.type = TINYGLTF_TYPE_SCALAR;
		accessor.count = pData->size();

		m_model.accessors.push_back(accessor);

		tinygltf::BufferView bufferView;

		// The combined byteOffset must be a multiple of the size of the component datatype.
		int type_size = sizeof(T);
		int alignment = type_size - pBuffer->data.size() % type_size;

		bufferView.byteOffset = pBuffer->data.size() + alignment;
		bufferView.buffer = bufferIndex;
		bufferView.byteLength = pData->size() * sizeof(T);

		m_model.bufferViews.push_back(bufferView);

		writeBuffer(&pBuffer->data, bufferView.byteOffset + accessor.byteOffset,
			accessor.count, false, bufferView.byteStride, pData);
	}

	template<class T>
	void writeAccessorData(const int bufferIndex, const int type, std::vector<std::vector<T>>* pData) {
		std::cout << "write accessor size: " << pData->size() << ", vec " << (*pData)[0].size() << std::endl;

		tinygltf::Buffer* pBuffer = &m_model.buffers[bufferIndex];

		tinygltf::Accessor accessor;

		int component_count = componentCount(type);

		bool is_matrix = componentIsMatrix(type);

		if (!is_matrix) {
			T n_min = std::numeric_limits<T>::max();
			T n_max = std::numeric_limits<T>::lowest();

			std::vector<T> min;
			std::vector<T> max;

			for (int i = 0; i < component_count; i++) {
				min.push_back(n_min);
				max.push_back(n_max);
			}

			for (int i = 0; i < pData->size(); i++) {
				auto element = (*pData)[i];

				for (int j = 0; j < component_count; j++) {
					T value = element[j];

					if (value < min[j]) min[j] = value;
					if (value > max[j]) max[j] = value;
				}
			}

			for (int i = 0; i < min.size(); i++) {
				accessor.minValues.push_back(min[i]);
				accessor.maxValues.push_back(max[i]);
			}
		}

		T t = 0;

		accessor.bufferView = m_model.bufferViews.size();
		accessor.byteOffset = (size_t)0;
		accessor.componentType = getGltfComponentType(t);
		accessor.type = type;
		accessor.count = pData->size();

		// std::cout << "DEBUG Accessor index: " << model.accessors.size() << " count: " << accessor.count << " type: " << accessor.type << std::endl;

		m_model.accessors.push_back(accessor);

		tinygltf::BufferView bufferView;

		// The combined byteOffset must be a multiple of the size of the component datatype.
		int type_size = sizeof(T);
		int alignment = type_size - pBuffer->data.size() % type_size;

		bufferView.byteOffset = pBuffer->data.size() + alignment;
		bufferView.buffer = bufferIndex;
		bufferView.byteLength = pData->size() * sizeof(T) * component_count;
		bufferView.byteStride = 0;

		m_model.bufferViews.push_back(bufferView);

		std::vector<T> simplified;
		for (auto v : *pData) {
			for (auto e : v) simplified.push_back(e);
		}

		writeBuffer(&pBuffer->data, bufferView.byteOffset + accessor.byteOffset, component_count, is_matrix, bufferView.byteStride, &simplified);
	}

	int writeSparseAccessorData(const int buffer_index, const int type, const std::vector<int32_t>* pIndices, const std::vector<std::vector<float>>* pData);
#pragma endregion

#pragma region write
	int writePrimitive(const T3DMesh<float>::Submesh* pSubmesh);

	void writeAttributes();

	std::pair<int, int> prepareAttributeArrays(const T3DMesh<float>::Submesh* pSubmesh);

	void writeMaterial(const T3DMesh<float>::Submesh* pSubmesh);

	int writeTexture(const std::string path);

	void writeNodes();

	void writeMorphTargets(std::pair<int, int> minmax);

	void writeSkinningData();

	void writeSkeletalAnimations();
#pragma endregion
};//GLTFIO

}//CForge
