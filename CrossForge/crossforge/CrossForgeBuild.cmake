cmake_minimum_required(VERSION 3.16)

SET(VCPKG_BOOTSTRAP_OPTIONS "-disableMetrics")    # Disable telemetry for vcpkg.
SET(X_VCPKG_APPLOCAL_DEPS_INSTALL ON)             # Install vcpkg dependencies automatically(experimental - might be changed or removed later; see: https://github.com/microsoft/vcpkg/issues/1653). 

# Forbid in-source builds.
IF("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
	MESSAGE(SEND_ERROR "In-source builds are not allowed. Use a different build directory.")
ENDIF("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")

# CrossForge library
project(crossforge VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(Optimization_Flag "-O2")


include(FetchContent)
FetchContent_Declare(
	stb 
	GIT_REPOSITORY https://github.com/nothings/stb.git
)
FetchContent_MakeAvailable(stb)
include_directories(${stb_SOURCE_DIR})

FetchContent_Declare(
	portable-file-dialogs
	GIT_REPOSITORY https://github.com/samhocevar/portable-file-dialogs
	GIT_TAG 0.1.0
)
FetchContent_MakeAvailable(portable-file-dialogs)
include_directories(${portable-file-dialogs_SOURCE_DIR}/)


if(EMSCRIPTEN)
	### Eigen3
	FetchContent_Declare(
		eigen3 
		GIT_REPOSITORY https://github.com/libigl/eigen.git
	)
	FetchContent_MakeAvailable(eigen3)
	include_directories(${eigen3_SOURCE_DIR}/)

	## WebP Library
	FetchContent_Declare(
		webp 
		GIT_REPOSITORY https://github.com/webmproject/libwebp.git
		GIT_TAG v1.3.0
	)
	FetchContent_MakeAvailable(webp)
	FetchContent_GetProperties(webp)

	include_directories(${webp_SOURCE_DIR}/src/)
	link_directories(${webp_BINARY_DIR})

	## Assimp
	FetchContent_Declare(
		assimp 
		URL https://github.com/assimp/assimp/archive/refs/tags/v5.2.5.zip
	)
	FetchContent_MakeAvailable(assimp)
	include_directories(${assimp_SOURCE_DIR}/include/)
	include_directories(${assimp_BINARY_DIR}/include/)
	link_directories(${assimp_BINARY_DIR}/lib/)

	## libigl
	FetchContent_Declare(
		libigl 
		URL https://github.com/libigl/libigl/archive/refs/tags/v2.4.0.zip
	)
	FetchContent_MakeAvailable(libigl)
	include_directories(${libigl_SOURCE_DIR}/include/)

	## freetype
	FetchContent_Declare(
		freetype 
		GIT_REPOSITORY https://github.com/freetype/freetype.git
		#GIT_TAG v2.0.3
	)
	FetchContent_MakeAvailable(freetype)
	include_directories(${freetype_SOURCE_DIR}/include/)

	# tinygltf
	FetchContent_Declare(
		tinygltf 
		URL https://github.com/syoyo/tinygltf/archive/refs/tags/v2.8.2.zip
	)
	FetchContent_MakeAvailable(tinygltf)
	include_directories(${tinygltf_SOURCE_DIR}/)

	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${Optimization_Flag} -fwasm-exceptions -Wno-deprecated -Wno-unused-command-line-argument -sUSE_ZLIB=1 -sUSE_GLFW=3 -sUSE_LIBPNG=1 -sUSE_LIBJPEG=1")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Optimization_Flag} -fwasm-exceptions -Wno-deprecated -Wno-unused-command-line-argument -Wno-tautological-pointer-compare -sUSE_ZLIB=1 -sUSE_GLFW=3 -sUSE_LIBPNG=1 -sUSE_LIBJPEG=1")

else()
	# required core packages
	FIND_PACKAGE(Eigen3 CONFIG REQUIRED)
	FIND_PACKAGE(OpenGL REQUIRED)		# OpenGl core library
	FIND_PACKAGE(glad CONFIG REQUIRED)	# GL extension library
	FIND_PACKAGE(glfw3 CONFIG REQUIRED)	# cross-plattform window management
	FIND_PACKAGE(assimp CONFIG REQUIRED)# Asset import library (and partially export)
	FIND_PACKAGE(freetype REQUIRED)		# Library to load and process vector based fonts
	FIND_PACKAGE(libigl CONFIG REQUIRED)# mesh processing library
	FIND_PACKAGE(WebP CONFIG REQUIRED)	# WebP to import/export webp
	find_package(JPEG REQUIRED)			# jpeg loader (required by libjpeg turbo)
	find_package(libjpeg-turbo CONFIG REQUIRED)			# jpeg-turbo library to compress/decompress jpeg
	FIND_PACKAGE(FFMPEG REQUIRED)

endif()

include_directories(
	"./"
)

add_library(crossforge SHARED
	# Core related
	crossforge/Core/CForgeObject.cpp
	crossforge/Core/CrossForgeException.cpp
	crossforge/Core/SCrossForgeDevice.cpp
	crossforge/Core/SGPIO.cpp
	crossforge/Core/SLogger.cpp
	crossforge/Core/SCForgeSimulation.cpp
	
	
	# Asset import/exporter stuff
	crossforge/AssetIO/File.cpp
	crossforge/AssetIO/AssimpMeshIO.cpp
	crossforge/AssetIO/I2DImageIO.cpp
	crossforge/AssetIO/I3DMeshIO.cpp
	crossforge/AssetIO/JPEGTurboIO.cpp
	crossforge/AssetIO/StbImageIO.cpp 
	crossforge/AssetIO/UserDialog.cpp
	crossforge/AssetIO/VideoRecorder.cpp
	crossforge/AssetIO/WebPImageIO.cpp
	crossforge/AssetIO/SAssetIO.cpp


	# Graphics related
	crossforge/Graphics/GBuffer.cpp 
	crossforge/Graphics/GLBuffer.cpp 
	crossforge/Graphics/GLVertexArray.cpp 
	crossforge/Graphics/GLWindow.cpp 
	crossforge/Graphics/RenderDevice.cpp 
	crossforge/Graphics/RenderMaterial.cpp  

	# Textures related
	crossforge/Graphics/Textures/GLCubemap.cpp 
	crossforge/Graphics/Textures/GLTexture2D.cpp 
	crossforge/Graphics/Textures/STextureManager.cpp

	# Camera related
	crossforge/Graphics/Camera/ViewFrustum.cpp
	crossforge/Graphics/Camera/VirtualCamera.cpp


	# Actor related
	crossforge/Graphics/Actors/IRenderableActor.cpp 
	crossforge/Graphics/Actors/RenderGroupUtility.cpp 
	crossforge/Graphics/Actors/SkyboxActor.cpp
	crossforge/Graphics/Actors/VertexUtility.cpp 
	crossforge/Graphics/Actors/ScreenQuad.cpp 
	crossforge/Graphics/Actors/StaticActor.cpp 
	crossforge/Graphics/Actors/SkeletalActor.cpp 
	crossforge/Graphics/Actors/MorphTargetActor.cpp 
	crossforge/Graphics/Actors/StickFigureActor.cpp

	# Animation Controller 
	crossforge/Graphics/Controller/SkeletalAnimationController.cpp 
	crossforge/Graphics/Controller/MorphTargetAnimationController.cpp

	# Shader
	crossforge/Graphics/Shader/GLShader.cpp 
	crossforge/Graphics/Shader/ShaderCode.cpp
	crossforge/Graphics/Shader/SShaderManager.cpp

	# Uniform Buffer Objects
	crossforge/Graphics/UniformBufferObjects/UBOCameraData.cpp 
	crossforge/Graphics/UniformBufferObjects/UBOColorAdjustment.cpp
	crossforge/Graphics/UniformBufferObjects/UBOLightData.cpp 
	crossforge/Graphics/UniformBufferObjects/UBOMaterialData.cpp 
	crossforge/Graphics/UniformBufferObjects/UBOModelData.cpp
	crossforge/Graphics/UniformBufferObjects/UBOBoneData.cpp 
	crossforge/Graphics/UniformBufferObjects/UBOMorphTargetData.cpp
	crossforge/Graphics/UniformBufferObjects/UBOTextData.cpp

	# Lights
	crossforge/Graphics/Lights/ILight.cpp 
	crossforge/Graphics/Lights/DirectionalLight.cpp 
	crossforge/Graphics/Lights/PointLight.cpp 
	crossforge/Graphics/Lights/SpotLight.cpp

	# SceneGraph
	crossforge/Graphics/SceneGraph/ISceneGraphNode.cpp
	crossforge/Graphics/SceneGraph/SceneGraph.cpp 
	crossforge/Graphics/SceneGraph/SGNGeometry.cpp
	crossforge/Graphics/SceneGraph/SGNTransformation.cpp

	# Font
	crossforge/Graphics/Font/Font.cpp
	crossforge/Graphics/Font/LineOfText.cpp
	crossforge/Graphics/Font/SFontManager.cpp

	# Math
	crossforge/Math/BoundingVolume.cpp
	crossforge/Math/CForgeMath.cpp

	# Input related
	crossforge/Input/Keyboard.cpp
	crossforge/Input/Mouse.cpp	
	crossforge/Input/SInputManager.cpp

	# Internet related
	crossforge/Network/UDPSocket.cpp
	crossforge/Network/TCPSocket.cpp

	# Mesh Processing
	crossforge/MeshProcessing/Builder/MorphTargetModelBuilder.cpp
	crossforge/MeshProcessing/PrimitiveShapeFactory.cpp


	# Utility
	crossforge/Utility/CForgeUtility.cpp

 )


if(EMSCRIPTEN)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Optimization_Flag}")
	add_compile_definitions(SHADER_GLES)
	target_link_libraries(crossforge 
		webp
		sharpyuv
		assimp
		freetype
		glfw
	)


elseif(WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W1 -wd4251")
add_compile_definitions(CFORGE_EXPORTS)

target_include_directories(crossforge 
	PRIVATE ${JPEG_INCLUDE_DIR}
	PRIVATE ${FFMPEG_INCLUDE_DIRS}
	)

	target_link_directories(crossforge
		PRIVATE ${FFMPEG_LIBRARY_DIRS}
	)

#TODO(skade) link release assimp
set_target_properties(assimp::assimp PROPERTIES MAP_IMPORTED_CONFIG_DEBUG Release)
target_link_libraries(crossforge 
	PRIVATE Eigen3::Eigen
	PRIVATE glfw 
	PRIVATE glad::glad
	PRIVATE assimp::assimp
	PRIVATE igl::igl_core
	PRIVATE WebP::webp
	PRIVATE WebP::webpdecoder
	ws2_32					#winsock2
	PRIVATE Freetype::Freetype
	libjpeg-turbo::turbojpeg
	PRIVATE ${JPEG_LIBRARIES}
	PRIVATE ${FFMPEG_LIBRARIES}
	)
elseif(__arm__)
	add_compile_definitions(USE_SYSFS_GPIO)
	target_link_libraries(crossforge 
	PRIVATE Eigen3::Eigen
	PRIVATE glfw
	PRIVATE glad::glad
	PRIVATE assimp::assimp
	PRIVATE igl::igl_core
	PRIVATE WebP::webp
	PRIVATE WebP::webpdecoder
	${FREETYPE_LIBRARIES}	# for Text rendering
	freetype

	PRIVATE gpiod 
	PRIVATE stdc++fs
	)
elseif(UNIX)

	target_link_libraries(crossforge 
	PRIVATE Eigen3::Eigen
	PRIVATE glfw
	PRIVATE glad::glad
	PRIVATE assimp::assimp
	PRIVATE igl::igl_core
	PRIVATE WebP::webp
	PRIVATE WebP::webpdecoder
	${FREETYPE_LIBRARIES}	# for Text rendering
	freetype
	PRIVATE stdc++fs
	PRIVATE libjpeg-turbo::turbojpeg-static
	PRIVATE ${JPEG_LIBRARIES}
	)
endif()

target_precompile_headers(crossforge PRIVATE
	pch.h
)

