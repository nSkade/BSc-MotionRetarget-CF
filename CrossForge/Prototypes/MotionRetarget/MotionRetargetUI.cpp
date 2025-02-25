#include "MotionRetargetScene.hpp"

#include <Prototypes/GUI/ImGuiUtility.h>
#include "UI/ImGuiStyle.hpp"
#include "Animation/IKSequencer.hpp"
#include <crossforge/AssetIO/UserDialog.h>
//TODOff(skade) for ImGuiUtility::initImGui replace
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>

#include <imgui_stdlib.h>

#include "AutoRig/ARpinocchio.hpp" //TODOff(skade) into Scene instead of here
#include "AutoRig/ARrignet.hpp" //TODOff(skade) into Scene instead of here

#include "AutoMoRe/MRlimb.hpp" //TODOff(skade) into Scene instead of here

#include "CMN/MergeVertices.hpp"

namespace ImGui {

auto ComboStr = [](const char* label, int* current_item, const std::vector<std::string>& items, int height_in_items = -1)
{
	return ImGui::Combo(label, current_item, [](void* data, int idx, const char** out_text) { *out_text = ((const std::vector<std::string>*)data)->at(idx).c_str(); return true; }, (void*)&items, (int) items.size(), height_in_items);
};

}//ImGui

namespace CForge {

void MotionRetargetScene::initUI() {
	//TODOff(skade) copied out of for now, order important?
	//ImGuiUtility::initImGui(&m_RenderWin);
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();

	ImGuiIO& io = ImGui::GetIO();

	io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // Enable Docking
	//io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;       // Enable Multi-Viewport / Platform Windows

	// When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular ones.
	//ImGuiStyle& style = ImGui::GetStyle();
	//if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
	//{
	//	style.WindowRounding = 0.0f;
	//	style.Colors[ImGuiCol_WindowBg].w = 1.0f;
	//}

	SetupImGuiStyle(true,0.5f);

	io.Fonts->AddFontDefault();

	ImGui_ImplGlfw_InitForOpenGL(static_cast<::GLFWwindow*>(m_RenderWin.handle()), true);
	ImGui_ImplOpenGL3_Init("#version 330 core");
}
void MotionRetargetScene::cleanUI() {
	ImGuiUtility::shutdownImGui();
}

void MotionRetargetScene::renderUI() {
	ImGuiUtility::newFrame();
	ImGuizmo::BeginFrame();
	ImGuiViewport* igViewPort = ImGui::GetMainViewport();
	ImGui::DockSpaceOverViewport(igViewPort,ImGuiDockNodeFlags_PassthruCentralNode);

	{ // transparent window for status infos
		ImGui::SetNextWindowBgAlpha(0.0);
		ImGui::Begin("status",0,ImGuiWindowFlags_::ImGuiWindowFlags_NoTitleBar);
		ImGui::Checkbox("EditMode",&m_isEditMode);

		bool moReLimb = m_MRlimb.active();
		if (moReLimb && ImGui::CollapsingHeader("MoRe")) {
			ImGui::Checkbox("Enabled",&moReLimb);
			ImGui::Checkbox("imitiate angle",&m_MRlimb.m_imitiateAngle);
			if (ImGui::CollapsingHeader("Root Options")) {
				ImGui::Checkbox("copy pos",&m_MRlimb.m_copy_rootPos);
				ImGui::Checkbox("copy rot",&m_MRlimb.m_copy_rootRot);
				ImGui::SliderFloat("pos scale",&m_MRlimb.m_scale_rootPos,0.,1.);
			}
			for (int i = 0; i < m_MRlimb.m_scale_limbs.size(); ++i)
				ImGui::SliderFloat(m_MRlimb.m_targets[i]->name.c_str(),&m_MRlimb.m_scale_limbs[i],0.,1.);
		}
		if (!moReLimb)
			m_MRlimb.reset();
		ImGui::End();
	}
	//ImGui::ShowDemoWindow();

	renderUI_menuBar();
	renderUI_Outliner();
	renderUI_animation();
	renderUI_Sequencer();
	renderUI_tools();
	renderUI_ik();
	renderUI_autorig();
	renderUI_autoMoRe();

	ImGuiUtility::render();

	//TODOfff(skade) move imgui outside glfw viewport, windows only
	//ImGuiIO& io = ImGui::GetIO();
	//if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
	//{
	//	::GLFWwindow* backup_current_context = glfwGetCurrentContext();
	//	ImGui::UpdatePlatformWindows();
	//	ImGui::RenderPlatformWindowsDefault();
	//	glfwMakeContextCurrent(backup_current_context);
	//}
}//renderUI

IKController::SkeletalJoint* MotionRetargetScene::renderUI_OutlinerJoints(std::shared_ptr<CharEntity> c, IKController::SkeletalJoint* selectedJoint) {

	std::function<void(CharEntity* c, IKController::SkeletalJoint* joint)> renderJointNode;
	IKController::SkeletalJoint* clickedNode = nullptr;

	renderJointNode = [&](CharEntity* c, IKController::SkeletalJoint* joint) {
		if (!joint)
			return;
	
		ImGuiTreeNodeFlags node_flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick | ImGuiTreeNodeFlags_SpanAvailWidth;
		if (joint == selectedJoint)
			node_flags |= ImGuiTreeNodeFlags_Selected;
		if (joint->Children.size() > 0) { // tree
			bool node_open = ImGui::TreeNodeEx((void*)(intptr_t)joint, node_flags,joint->Name.c_str());
			if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
				clickedNode = joint;
			if (node_open) {
				for (uint32_t i=0;i<joint->Children.size();++i)
					renderJointNode(c,c->controller->getBone(joint->Children[i]));
				ImGui::TreePop();
			}
		}
		else { // leaf
			node_flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen; // ImGuiTreeNodeFlags_Bullet
			//ImGui::TreeNodeEx((void*)(intptr_t)i, node_flags, "Selectable Leaf %d", i);
			ImGui::TreeNodeEx((void*) (intptr_t)joint,node_flags,joint->Name.c_str());
			if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
				clickedNode = joint;
		}
	};
	renderJointNode(c.get(),c->controller->getRoot());
	return clickedNode;
}

void MotionRetargetScene::renderUI_Outliner() {

	ImGui::Begin("Outliner");
	{ // currently selected item
		std::string item = "";
		if (auto lp = m_picker.getLastPick().lock()) {
			if (auto p = std::dynamic_pointer_cast<CharEntity>(lp))
				item = "CharEntity "+p->name;
			if (auto p = std::dynamic_pointer_cast<JointPickable>(lp))
				item = "Joint "+p->m_pJoint->Name;
			if (auto p = std::dynamic_pointer_cast<IKTarget>(lp))
				item = "Target "+p->name;
		}
		ImGui::Text(item.c_str());
	}
	ImGui::Separator();

	//TODOff(skade) make selectable
	for (uint32_t i=0;i<m_charEntities.size();++i) {
		auto c = m_charEntities[i];
		if (!c)
			continue;
		if (ImGui::TreeNode(c->name.c_str())) {
			ImGui::SameLine();
			if (ImGui::Button("Pick")) {
				forcePickCharEntity(c);
			}

			if (ImGui::CollapsingHeader("visiblity options")) {
				bool onSG,onSG2; c->sgn.enabled(&onSG,&onSG2);
				ImGui::Checkbox("onSG",&onSG);
				c->sgn.enable(onSG,onSG);
				ImGui::SameLine();
				ImGui::SliderFloat("visibility",&c->visibility,0.,1.);

				if (c->controller) {
					//ImGui::SameLine();
					auto jps = c->controller->getJointPickables();
					float jpo = jps[0].lock()->getOpacity();
					ImGui::SetNextItemWidth(ImGui::GetWindowWidth()*.5);
					ImGui::DragFloat("Joint Opacity",&jpo,.005,0.,1.);
					for (auto jp : jps)
						jp.lock()->setOpacity(jpo);

					//TODOff(skade) improve usage
					// need to set highlight behavior for all joints
					static bool hullHighlight = true;
					ImGui::Checkbox("hull highlight",&hullHighlight);
					for (auto jp : jps)
						jp.lock()->highlightBehind = hullHighlight;
				}
			} // visibility
			if (!c->controller) {
				ImGui::TreePop();
				continue;
			}
			if (ImGui::TreeNode("Armature")) {
				IKController::SkeletalJoint* clickedNode = renderUI_OutlinerJoints(c,m_outlinerSelJoint);

				if (clickedNode) {
					if (ImGui::GetIO().KeyCtrl) { // CTRL+click to toggle
						if (m_outlinerSelJoint == clickedNode) {
							m_outlinerSelJoint = nullptr;
							m_picker.reset();
						}
						else {
							m_outlinerSelJoint = clickedNode;
							m_picker.forcePick(c->controller->getJointPickable(clickedNode));
							m_guizmoMat = m_picker.m_guizmoMat;
						}
					}
					else {
						m_outlinerSelJoint = clickedNode;
						m_picker.forcePick(c->controller->getJointPickable(clickedNode));
						m_guizmoMat = m_picker.m_guizmoMat;
					}
				}
				ImGui::TreePop();
			} // Armature
			if (ImGui::TreeNode("Targets")) {
				for (uint32_t j = 0; j < c->controller->m_targets.size(); ++j) {
					auto t = c->controller->m_targets[j];
					ImGuiTreeNodeFlags node_flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick | ImGuiTreeNodeFlags_SpanAvailWidth;
					node_flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen; // ImGuiTreeNodeFlags_Bullet
					ImGui::TreeNodeEx(t->name.c_str(),node_flags);
					if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) {
						m_picker.forcePick(t);
						m_guizmoMat = m_picker.m_guizmoMat;
					}
				}
				ImGui::TreePop();
			} // Targets
			ImGui::TreePop();
		} // if main tree node
	} // for charEntities
	
	ImGui::End();
}

void MotionRetargetScene::renderUI_animation() {
	ImGui::Begin("Animation");
	auto c = m_charEntityPrim.lock();

	if (!c || !c->actor.get()) {
		ImGui::End();
		return;
	}

	std::vector<std::string> items(c->controller->animationCount()+1);
	items[0] = "none";
	for (uint32_t i=0;i<c->controller->animationCount();++i)
		items[i+1] = c->controller->animation(i)->Name;
	ImGui::ComboStr("select animation",&c->animIdx,items);

	if (c->animIdx > 0) { // anim selected

		//TODOff(skade) move into char entity?
		if (c->pAnimCurr && c->animIdx - 1 != c->pAnimCurr->AnimationID) { // Animation changed
			c->controller->destroyAnimation(c->pAnimCurr);
			c->actor->activeAnimation(nullptr);
			c->pAnimCurr = nullptr;
		}
		if (!c->pAnimCurr) { // create animation if not existing
			c->pAnimCurr = c->controller->createAnimation(c->animIdx-1,1.f,0.f);
			c->actor->activeAnimation(c->pAnimCurr);
		}

		T3DMesh<float>::SkeletalAnimation* anim = c->actor->getController()->animation(c->animIdx-1);
		ImGui::Text("Duration: %f",anim->Duration);
		ImGui::SameLine();
		ImGui::Text("SamplesPerSecond: %f",anim->SamplesPerSecond);
		if (!c->m_animAutoplay) {
			if(ImGui::Button("Play")) {
				c->m_animAutoplay = true;
			}
		} else {
			if(ImGui::Button("Stop")) {
				c->m_animAutoplay = false;
			}
			ImGui::SameLine();
			ImGui::DragFloat("animSpeed", &(c->pAnimCurr->Speed), 0.01f);
		}
		ImGui::Text("pAnim->t: %f",c->pAnimCurr->t);
	}
	else {
		c->controller->destroyAnimation(c->pAnimCurr);
		c->actor->activeAnimation(nullptr);
		c->pAnimCurr = nullptr;
	}
	ImGui::End();
}

void MotionRetargetScene::renderUI_Sequencer() {
	ImGui::Begin("Sequencer");

	// sequence with default values
	//TODOff(skade) make member
	static MySequence mySequence;
	static bool init = false;
	if (!init) {
		mySequence.mFrameMin = 0;
		mySequence.mFrameMax = 1000;
		//TODOff(skade)
		//mySequence.myItems.push_back(MySequence::MySequenceItem{ 0, 0, 10, false});
		//mySequence.myItems.push_back(MySequence::MySequenceItem{ 0, 0, int(pAnim->Duration*pAnim->SamplesPerSecond), false });
		//mySequence.myItems.push_back(MySequence::MySequenceItem{ 1, 20, 30, true });
		//mySequence.myItems.push_back(MySequence::MySequenceItem{ 3, 12, 60, false });
		//mySequence.myItems.push_back(MySequence::MySequenceItem{ 2, 61, 90, false });
		//mySequence.myItems.push_back(MySequence::MySequenceItem{ 4, 90, 99, false });
		init = true;
	}

	int animFrameCurr = 0;
	mySequence.myItems.clear();
	if (auto c = m_charEntityPrim.lock()) {
		if (c->pAnimCurr) {
			int animFrameCount = c->pAnimCurr->Duration * c->pAnimCurr->SamplesPerSecond;
			//int animFrameCount = c->controller->animation(c->pAnimCurr->AnimationID)->Keyframes[0]->Positions.size(); //TODOf(skade)
			animFrameCount -= 1; // visualizer is inclusive
			mySequence.myItems.push_back(MySequence::MySequenceItem{0,0,animFrameCount,false});
			animFrameCurr = c->animFrameCurr;
		}
	}

	ImGui::PushItemWidth(130);
	ImGui::Text("Frame");
	ImGui::SameLine();
	ImGui::InputInt("\t\t", &animFrameCurr);
	ImGui::SameLine();
	ImGui::Text("Min");
	ImGui::SameLine();
	ImGui::InputInt("\t\t", &mySequence.mFrameMin);
	ImGui::SameLine();
	ImGui::Text("Max");
	ImGui::SameLine();
	ImGui::InputInt("\t\t", &mySequence.mFrameMax);
	ImGui::PopItemWidth();
	ImGui::SameLine();
	ImGui::Text("Keyframe: ");
	ImGui::SameLine();
	if (ImGui::Button("Set")) {

	}
	ImGui::SameLine();
	if (ImGui::Button("Get")) {

	}

	//TODOff(skade) make member
	static int selectedEntry = -1;
	static int firstFrame = 0;
	static bool expanded = true;
	Sequencer(&mySequence, &animFrameCurr, &expanded, &selectedEntry, &firstFrame,
	          ImSequencer::SEQUENCER_EDIT_STARTEND | ImSequencer::SEQUENCER_ADD | ImSequencer::SEQUENCER_DEL |
	          ImSequencer::SEQUENCER_COPYPASTE | ImSequencer::SEQUENCER_CHANGE_FRAME);
	if (auto c = m_charEntityPrim.lock()) {
		c->animFrameCurr = animFrameCurr;
		if (c->animFrameCurr > mySequence.mFrameMax)
			c->animFrameCurr = 0;
	}
	
	//TODOff(skade)
	// add a UI to edit that particular item
	//if (selectedEntry != -1) {
	//	const MySequence::MySequenceItem &item = mySequence.myItems[selectedEntry];
	//	ImGui::Text("I am a %s, please edit me", SequencerItemTypeNames[item.mType]);
	//	// switch (type) ....
	//}
	ImGui::End();
}

void MotionRetargetScene::renderUI_menuBar() {
	if (ImGui::BeginMainMenuBar()) {
		if (ImGui::BeginMenu("File"))
		{
			if (ImGui::BeginMenu("Import")) {
				if (ImGui::MenuItem("GLTF", ".gltf, .glb")) {
					std::string path = UserDialog::OpenFile("load primary char", "gltf", "*.gltf *.glb");
					loadCharPrim(path,IOM_GLTFIO);
				}
				if (ImGui::MenuItem("Assimp")) {
					std::string path = UserDialog::OpenFile("load primary char", "assimp");
					loadCharPrim(path,IOM_ASSIMP);
				}
				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("Export")) {
				if (ImGui::MenuItem("GLTF", ".gltf, .glb")) {
					std::string path = UserDialog::SaveFile("store primary char", "gltf", "*.gltf *.glb");
					storeCharPrim(path,IOM_GLTFIO);
				}
				if (ImGui::MenuItem("Assimp")) {
					std::string path = UserDialog::SaveFile("store primary char", "assimp");
					storeCharPrim(path,IOM_ASSIMP);
				}
				if (ImGui::MenuItem("objImport")) {
					std::string path = UserDialog::SaveFile("store primary char", "objImport", "*.obj");
					storeCharPrim(path,IOM_OBJIMP);
				}
				ImGui::EndMenu();
			}

//
			
			//if (ImGui::BeginMenu("Open Recent"))
			//{
			//	ImGui::MenuItem("fish_hat.c");
			//	ImGui::MenuItem("fish_hat.inl");
			//	ImGui::MenuItem("fish_hat.h");
			//	if (ImGui::BeginMenu("More.."))
			//	{
			//		ImGui::MenuItem("Hello");
			//		ImGui::MenuItem("Sailor");
			//		if (ImGui::BeginMenu("Recurse.."))
			//		{
			//			//ShowExampleMenuFile();
			//			ImGui::EndMenu();
			//		}
			//		ImGui::EndMenu();
			//	}
			//	ImGui::EndMenu();
			//}
			//if (ImGui::MenuItem("Save", "Ctrl+S")) {}
			//if (ImGui::MenuItem("Save As..")) {}

			//ImGui::Separator();
			////IMGUI_DEMO_MARKER("Examples/Menu/Options");
			//if (ImGui::BeginMenu("Options"))
			//{
			//	static bool enabled = true;
			//	ImGui::MenuItem("Enabled", "", &enabled);
			//	ImGui::BeginChild("child", ImVec2(0, 60), true);
			//	for (int i = 0; i < 10; i++)
			//		ImGui::Text("Scrolling Text %d", i);
			//	ImGui::EndChild();
			//	static float f = 0.5f;
			//	static int n = 0;
			//	ImGui::SliderFloat("Value", &f, 0.0f, 1.0f);
			//	ImGui::InputFloat("Input", &f, 0.1f);
			//	ImGui::Combo("Combo", &n, "Yes\0No\0Maybe\0\0");
			//	ImGui::EndMenu();
			//}

			////IMGUI_DEMO_MARKER("Examples/Menu/Colors");
			//if (ImGui::BeginMenu("Colors"))
			//{
			//	float sz = ImGui::GetTextLineHeight();
			//	for (int i = 0; i < ImGuiCol_COUNT; i++)
			//	{
			//		const char* name = ImGui::GetStyleColorName((ImGuiCol)i);
			//		ImVec2 p = ImGui::GetCursorScreenPos();
			//		ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x + sz, p.y + sz), ImGui::GetColorU32((ImGuiCol)i));
			//		ImGui::Dummy(ImVec2(sz, sz));
			//		ImGui::SameLine();
			//		ImGui::MenuItem(name);
			//	}
			//	ImGui::EndMenu();
			//}

			//// Here we demonstrate appending again to the "Options" menu (which we already created above)
			//// Of course in this demo it is a little bit silly that this function calls BeginMenu("Options") twice.
			//// In a real code-base using it would make senses to use this feature from very different code locations.
			//if (ImGui::BeginMenu("Options")) // <-- Append!
			//{
			//	//IMGUI_DEMO_MARKER("Examples/Menu/Append to an existing menu");
			//	static bool b = true;
			//	ImGui::Checkbox("SomeOption", &b);
			//	ImGui::EndMenu();
			//}

			//if (ImGui::BeginMenu("Disabled", false)) // Disabled
			//{
			//	IM_ASSERT(0);
			//}


			//if (ImGui::MenuItem("Checked", NULL, true)) {}
//
			if (ImGui::MenuItem("Quit", "ESC"))
				m_exitCalled = true;
			ImGui::EndMenu();
		}
		//if (ImGui::BeginMenu("View")) {
		//	if (ImGui::MenuItem("Load cesium man"))
		//		initCesiumMan();
		//	ImGui::EndMenu();
		//}

		if (ImGui::BeginMenu("Tools")) {
			if (ImGui::MenuItem("merge redundent vertices")) {
				if (auto c = m_charEntityPrim.lock()) {
					mergeRedundantVertices(&c->mesh);
					c->init(&m_sgnRoot);
				}
				m_picker.reset();
			}
			if (ImGui::MenuItem("Apply Transformation to Mesh Data")) {
				if (auto c = m_charEntityPrim.lock())
					c->applyTransformToMesh(&m_sgnRoot);
				m_picker.reset();
			}
			if (ImGui::MenuItem("assign current pose as restpose")) {
				if (auto c = m_charEntityPrim.lock())
					c->updateRestpose(&m_sgnRoot);
				m_picker.reset();
			}
			if (ImGui::MenuItem("current pose to restpose")) {
				if (auto c = m_charEntityPrim.lock()) {
					if (c->controller)
						c->controller->initRestpose();
				}
				m_picker.reset();
			}
			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Armature")) {
			if (ImGui::MenuItem("Import Armature config")) {
				if (auto c = m_charEntityPrim.lock()) {
					std::filesystem::path path = UserDialog::OpenFile("select armature json", "json", "*.json");
					if (!path.empty()) {
						c->importArmature(path);
						try {
							c->parseArmature();
						}
						catch (...) {
							std::cerr << "error parsing armature, make sure joint names are correct"; //TODOfff(skade) proper log
						}
					}
				}
			}
			if (ImGui::MenuItem("Export Armature config")) {
				if (auto c = m_charEntityPrim.lock()) {
					std::filesystem::path path = UserDialog::SaveFile("store armature json", "json", "*.json");
					c->extractArmature();
					c->exportArmature(path);
				}
			}
			if (ImGui::MenuItem("create Targets for limbs")) {
				if (auto c = m_charEntityPrim.lock())
					c->autoCreateTargets();
			}
			if (ImGui::MenuItem("remove Armature")) {
				if (auto c = m_charEntityPrim.lock())
					c->removeArmature(&m_sgnRoot);
				m_picker.reset();
			}
			if (ImGui::MenuItem("auto create Armature")) {
				if (auto c = m_charEntityPrim.lock())
					c->autoCreateArmature();
				m_picker.reset();
			}
			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("AutoRig")) {
			if (ImGui::MenuItem("rignet"))
				m_showPop[POP_AR_RIGNET] = true;
			if (ImGui::MenuItem("pinocchio"))
				m_showPop[POP_AR_PINOC] = true;
			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("AutoMoRe")) {
			if (ImGui::MenuItem("limb"))
				m_showPop[POP_MR_LIMB] = true;
			//if (ImGui::MenuItem("pinocchio"))
			//	m_showPop[POP_AR_PINOC] = true;
			ImGui::EndMenu();
		}


		//TODOff(skade) move logic outside
		if (ImGui::BeginMenu("Options")) {
			if (ImGui::MenuItem("Reset Camera")) {
				Vector3f c = Vector3f(.5,0.,-.5);
				m_Cam.lookAt(Vector3f(4.,2.5,4.)+c,c);
			}
			if (ImGui::MenuItem("Preferences"))
				m_showPop[POP_PREF] = true;
			
			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu("Debug")) {
			if (ImGui::MenuItem("Load cesium man"))
				initCesiumMan();
			ImGui::EndMenu();
		}

		ImGui::EndMainMenuBar();

		if (m_showPop[POP_PREF]) {
			ImGui::SetNextWindowSize(ImVec2(520, 600), ImGuiCond_FirstUseEver);
			bool popState = m_showPop[POP_PREF];
			if (ImGui::Begin("Preferences", &popState)) {
				ImGui::BeginChild("item view", ImVec2(0, -ImGui::GetFrameHeightWithSpacing())); {
					ImGui::Text("Grid");
					ImGui::Checkbox("Render Debug Grid",&m_settings.renderDebugGrid);
					ImGui::InputScalar("Grid Size",ImGuiDataType_Float,&m_settings.gridSize);
					ImGui::Separator();
					ImGui::Text("Debug");
					ImGui::Checkbox("Render Debug Cube",&m_guizmo.m_renderDebugCube);
					ImGui::Separator();
					ImGui::Separator();
					ImGui::Checkbox(m_cesStartupStr.c_str(),&m_settings.cesStartup);

					if (ImGui::Button("path Conda")) {
						m_settings.pathAnaconda = UserDialog::SelectFolder("path Conda");
					}
					ImGui::SameLine();
					ImGui::Text(m_settings.pathAnaconda.c_str());
					ImGui::Text("path to anaconda installation, folder which should contain _conda.exe");

					if (ImGui::Button("path Rignet")) {
						m_settings.pathRignet = UserDialog::SelectFolder("path Rignet");
					}
					ImGui::SameLine();
					ImGui::Text(m_settings.pathRignet.c_str());
					ImGui::Text("path to rignet root, folder which should contain quick_start.py");
				} ImGui::EndChild();
				
				if (ImGui::Button("Save Settings")) {
					m_config.store(m_cesStartupStr.c_str(), m_settings.cesStartup);
					m_config.store("path.anaconda", m_settings.pathAnaconda);
					m_config.store("path.rignet", m_settings.pathRignet);
					m_config.baseStore();
					popState = false;
				}
			}
			m_showPop[POP_PREF] = popState;
			ImGui::End();
		}
	}
}//renderUI_menuBar

void MotionRetargetScene::renderUI_tools() {
	ImGui::Begin("Tools");
	if (ImGui::CollapsingHeader("Visualizers", ImGuiTreeNodeFlags_None)) {
		ImGui::Checkbox("Show Joints", &m_settings.showJoints);
		ImGui::SameLine();
		ImGui::Checkbox("Show Targets", &m_settings.showTargets);
		//TODOfff(skade)
		//if (m_FPSLabelActive)
		//	m_FPSLabel.render(&m_RenderDev);
		//if (m_DrawHelpTexts)
		//	drawHelpTexts();
	}

	if (ImGui::CollapsingHeader("Guizmo", ImGuiTreeNodeFlags_Selected)) {
		m_guizmo.renderOptions();
	}
	m_guizmo.setMat(&m_guizmoMat);
	m_guizmo.setCam(&m_Cam);
	m_guizmo.render();

	ImGuizmo::SetRect(0, 0, m_RenderWin.width(), m_RenderWin.height());
	Eigen::Matrix4f cameraMat = m_Cam.cameraMatrix();
	float* cameraView = cameraMat.data();
	float* cameraProjection = m_Cam.projectionMatrix().data();
	Eigen::Matrix4f identityMatrix = Eigen::Matrix4f::Identity();

	float camDistance = cameraMat.block<3,1>(0,3).norm();

	//TODOff(skade) improve focus point
	Matrix4f oldView = cameraMat;
	m_viewManipulate.render(&cameraMat,camDistance);

	if (oldView != cameraMat)
		m_guizmoViewManipChanged = true;
	else
		m_guizmoViewManipChanged = false;
	m_Cam.cameraMatrix(cameraMat);

	//TODOff(skade) old imguizmo grid, remove
	//if (m_settings.renderDebugGrid) {
	//	ImGuizmo::DrawGrid(cameraView, cameraProjection, identityMatrix.data(), m_settings.gridSize);
	//}

	ImGui::End();
}

void MotionRetargetScene::renderUI_ik() {
	ImGui::Begin("IK");

	auto c = m_charEntityPrim.lock();
	if (!c || !c->controller) {
		ImGui::End();
		return;
	}
	static CharEntity* prevC = nullptr;
	if (prevC != c.get()) { // m_charEntityPrim changed
		m_selChainIdx = -1;
		m_selChainIdxPrev = -1;
	}
	prevC = c.get();

	std::vector<IKChain>& chains = c->controller->getJointChains();

	{ // ik method
		ImGui::Checkbox("enable IK",&c->m_IKCupdate);
		ImGui::SameLine();
		if (ImGui::Button("singleIK"))
			c->m_IKCupdateSingle = true;

		const std::vector<std::string> ikMstr = {
			"IKSS_NONE",
			"IKSS_CCD",
			"IKSS_FABRIK",
			"IKSS_JACINV"
		};
		enum IKMethod {
			IKSS_NONE = 0,
			IKSS_CCD,
			IKSS_FABRIK,
			IKSS_JACINV
		};

		auto ikToIdx = [](IIKSolver* s) -> IKMethod {
			if (auto iksCCD = dynamic_cast<IKSccd*>(s))
				return IKSS_CCD;
			else if (dynamic_cast<IKSfabrik*>(s))
				return IKSS_FABRIK;
			else if (dynamic_cast<IKSjacInv*>(s))
				return IKSS_JACINV;
			return IKSS_NONE;
		};
		auto makeIK = [](IKChain* c, IKMethod m) {
			switch (m)
			{
			default:
			case IKSS_NONE:
				c->ikSolver.reset();
				break;
			case IKSS_CCD:
				c->ikSolver = std::make_unique<IKSccd>();
				break;
			case IKSS_FABRIK:
				c->ikSolver = std::make_unique<IKSfabrik>();
				break;
			case IKSS_JACINV:
				c->ikSolver = std::make_unique<IKSjacInv>();
				break;
			}
		};

		if (m_selChainIdx != -1) {
			IKChain& chain = chains[m_selChainIdx];
			IKMethod idx = ikToIdx(chain.ikSolver.get());
			IKMethod prevIdx = idx;

			//TODOfff(skade) cleaner impl with enums
			ImGui::ComboStr("ik method",(int*) &idx,ikMstr);
			if (prevIdx != idx) {
				makeIK(&chain,idx);
			}
			int subType = 0;
			if (auto iks = dynamic_cast<IKSjacInv*>(chain.ikSolver.get())) {
				const std::vector<std::string> meth = {
					"TRANSPOSE",
					"SVD (unstable)",
					"DLS",
				};
				subType = iks->m_type;
				ImGui::ComboStr("inv meth",&subType,meth);
				iks->m_type = (IKSjacInv::Type) subType;
			}
			if (auto iks = dynamic_cast<IKSccd*>(chain.ikSolver.get())) {
				const std::vector<std::string> meth = {
					"BACKWARD",
					"FORWARD",
				};
				subType = iks->m_type;
				ImGui::ComboStr("inv meth",&subType,meth);
				iks->m_type = (IKSccd::Type) subType;
			}
			if (auto iks = dynamic_cast<IIKSolver*>(chain.ikSolver.get())) {
				if (ImGui::CollapsingHeader("CMN opt")) {
					ImGui::InputInt("maxIt",&iks->m_MaxIterations);
					ImGui::InputFloat("thDist",&iks->m_thresholdDist     ,0.f,0.f,"%.10f");
					ImGui::InputFloat("thDelt",&iks->m_thresholdPosChange,0.f,0.f,"%.10f");
				}
			}

			if (ImGui::Button("all chains to curr setup")) {
				for (auto& c : chains) {
					makeIK(&c,idx);
					if (auto iks = dynamic_cast<IKSccd*>(c.ikSolver.get()))
						iks->m_type = (IKSccd::Type) subType;
					if (auto iks = dynamic_cast<IKSjacInv*>(c.ikSolver.get()))
						iks->m_type = (IKSjacInv::Type) subType;
				}
			}
		}
	}

	if (ImGui::BeginListBox("IK Chains")) {
		for (int n = 0; n < chains.size(); n++) {
			const bool is_selected = (m_selChainIdx == n);
			if (ImGui::Selectable(chains[n].name.c_str(), is_selected)) {
				if (ImGui::GetIO().KeyCtrl) { // CTRL+click to toggle
					if (m_selChainIdx == n)
						m_selChainIdx = -1;
				}
				else
					m_selChainIdx = n;
			}

			if (ImGui::IsItemActive() && !ImGui::IsItemHovered()) {
				int n_next = n + (ImGui::GetMouseDragDelta(0).y < 0.f ? -1 : 1);
				if (n_next >= 0 && n_next < chains.size()) {
					std::swap(chains[n],chains[n_next]);
					ImGui::ResetMouseDragDelta();
				}
			}

			// Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
			if (is_selected)
				ImGui::SetItemDefaultFocus();
		}
		ImGui::EndListBox();
	}

	//TODOff(skade) more efficient?
	//for (uint32_t i = 0; i < items.size(); ++i) {
	//	ikChains[items[i]].
	//}

	//TODOff(skade) adjust when changing to other rigged char
	// highlight chain joints green
	if (m_selChainIdx != -1) {
		auto js = chains[m_selChainIdx].joints;
		for (uint32_t i = 0; i < js.size(); ++i) {
			auto jp = c->controller->getJointPickable(js[i]).lock();
			jp->colorSelect = Vector4f(0.,1.,0.,1.);
			jp->m_highlight = true;
		}
	}
	if (m_selChainIdxPrev != m_selChainIdx && m_selChainIdxPrev != -1) {
		auto js = chains[m_selChainIdxPrev].joints;
		for (uint32_t i = 0; i < js.size(); ++i) {
			auto jp = c->controller->getJointPickable(js[i]).lock();
			jp->colorSelect = jp->colorSelect0;
			jp->m_highlight = false;
		}
	}
	m_selChainIdxPrev = m_selChainIdx;
	
	if (ImGui::CollapsingHeader("IKChain operations")) {
		//if (m_selChainIdx != -1) {
		//	auto& chain = chains[m_selChainIdx];
		//	ImGui::DragFloat("weight",&chain.weight,.005,0.,1.);
		//}

		renderUI_ikChainEditor(&m_selChainIdx);

		std::string targetName = "none";
		if (m_selChainIdx != -1)
			if (auto t = chains[m_selChainIdx].target.lock())
				targetName = t->name;
		if (ImGui::Button("set target")) {
			if (auto p = std::dynamic_pointer_cast<IKTarget>(m_picker.getLastPick().lock())) {
				chains[m_selChainIdx].target = p; //TODOff(skade) targets need to be smart ptr when decoupled
				// problem when p is from other charEntity
			}
		}
		ImGui::SameLine();
		ImGui::Text((std::string("target: ") + targetName).c_str());
		
		//TODOff(skade) seperate logic from ui
		if (ImGui::Button("delete selected chain")) {
			if (chains.size() > 0 && m_selChainIdx != -1) {

				auto js = chains[m_selChainIdx].joints;
				for (uint32_t i = 0; i < js.size(); ++i) {
					auto jp = c->controller->getJointPickable(js[i]).lock();
					jp->colorSelect = jp->colorSelect0;
					jp->m_highlight = false;
				}

				chains.erase(chains.begin() + m_selChainIdx);
				
				m_selChainIdx = -1;
				m_selChainIdxPrev = -1;
			}
		}
	}
	if (ImGui::CollapsingHeader("Target operations")) {
		renderUI_ikTargetEditor();
	}
	ImGui::End();
}

void MotionRetargetScene::renderUI_ikChainEditor(int* item_current_idx) {
	if (ImGui::Button("edit chain"))
		m_showPop[POP_CHAINED] = true;

	auto c = m_charEntityPrim.lock();
	if (!c || !c->controller) {
		ImGui::End();
		return;
	}
	static CharEntity* prevC = nullptr;
	if (prevC != c.get()) { // m_charEntityPrim changed
		m_ikceNameInit = false;
		m_ikceName = "new";
		m_ikceRootJoint = nullptr;
		m_ikceEndEffJoint = nullptr;
	}
	prevC = c.get();
	auto& chains = c->controller->getJointChains();

	if (m_showPop[POP_CHAINED]) {
		// Always center this window when appearing
		ImVec2 center = ImGui::GetMainViewport()->GetCenter();
		ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

		//TODO(skade) on popup creation fetch currently set chain root and eef if existing, use init bool for it resetting on confirmation

		bool popState = m_showPop[POP_CHAINED];
		if (ImGui::Begin("add ik chain", &popState)) {
			if (!m_ikceNameInit) {
				if (*item_current_idx != -1)
					m_ikceName = chains[*item_current_idx].name;
				m_ikceNameInit = true;
			}
			ImGui::InputText("chain name:", &m_ikceName);

			ImVec2 subSize = ImGui::GetWindowSize();
			subSize.y *= .4;

			{
				ImGui::BeginChild("root joint",subSize,true);
				std::string rootJointName = "none";
				if (m_ikceRootJoint)
					rootJointName = m_ikceRootJoint->Name;
				ImGui::Text((std::string("root joint: ") + rootJointName).c_str());
				ImGui::Separator();
				IKController::SkeletalJoint* clickedJoint = renderUI_OutlinerJoints(c,m_ikceRootJoint);

				// change color
				if (clickedJoint) {
					if (m_ikceRootJoint) {
						auto jp = c->controller->getJointPickable(m_ikceRootJoint).lock();
						jp->restoreColor();
						jp->m_highlight = false;
					}
					m_ikceRootJoint = clickedJoint;
					auto jp = c->controller->getJointPickable(m_ikceRootJoint).lock();
					jp->colorSelect = Vector4f(1.,0.,0.,1.);
					jp->m_highlight = true;
				}
				ImGui::EndChild();
			}

			{
				ImGui::BeginChild("endEff joint",subSize, true);
				std::string endEffJointName = "none";
				if (m_ikceEndEffJoint)
					endEffJointName = m_ikceEndEffJoint->Name;
				ImGui::Text((std::string("endEff joint: ") + endEffJointName).c_str());
				ImGui::Separator();
				IKController::SkeletalJoint* clickedJoint = renderUI_OutlinerJoints(c,m_ikceEndEffJoint);
				if (clickedJoint) {
					if (m_ikceEndEffJoint) {
						auto jp = c->controller->getJointPickable(m_ikceEndEffJoint).lock();
						jp->restoreColor();
						jp->m_highlight = false;
					}
					m_ikceEndEffJoint = clickedJoint;
					auto jp = c->controller->getJointPickable(m_ikceEndEffJoint).lock();
					jp->colorSelect = Vector4f(0.,0.,1.,1.);
					jp->m_highlight = true;
				}
				ImGui::EndChild();
			}

			if (ImGui::Button("Confirm")) {
				//IKChain nChain = c->controller->getJointChains()[m_ikceName];
				IKChain* nChain = c->controller->getIKChain(m_ikceName);
				if (!nChain) {
					nChain = &(c->controller->getJointChains().emplace_back());
					//nChain = &c->controller->getJointChains().back(); //TODOf(skade)
				}
				nChain->name = m_ikceName;
				
				nChain->joints.clear();
				{
					IKController::SkeletalJoint* j = m_ikceEndEffJoint;
					nChain->joints.push_back(j);
					do {
						j = c->controller->getBone(j->Parent);
						nChain->joints.push_back(j);
					} while (j != m_ikceRootJoint);
				}
				
				//nChain->pRoot = &c->controller->m_IKJoints[m_ikceRootJoint]; //TODO(skade) unused
				//c->controller->getJointChains()[m_ikceName] = nChain;
				
				m_ikceName = "new"; m_ikceNameInit = false;
				m_showPop[POP_CHAINED] = false;
			}
			ImGui::End();
		}
		m_showPop[POP_CHAINED] = popState;
	} // if addChainPopup
	else {
		m_ikceName = "new"; m_ikceNameInit = false;
		//TODO potential bug rootJoint still from other charEntity on swap
		if (m_ikceRootJoint) {
			auto jp = c->controller->getJointPickable(m_ikceRootJoint).lock();
			if (jp) {
				jp->restoreColor();
				jp->m_highlight = false;
			}
		}
		if (m_ikceEndEffJoint) {
			auto jp = c->controller->getJointPickable(m_ikceEndEffJoint).lock();
			if (jp) {
				jp->restoreColor();
				jp->m_highlight = false;
			}
		}
	}
}
void MotionRetargetScene::renderUI_ikTargetEditor() {
	auto c = m_charEntityPrim.lock();
	if (!c || !c->controller) {
		ImGui::End();
		return;
	}
	auto& chains = c->controller->getJointChains();
	static std::string newTargetName = "new target";
	ImGui::InputText("new target name",&newTargetName);
	if (ImGui::Button("add new target")) {
		std::string name = newTargetName;
		BoundingVolume bv;
		Vector3f d =Vector3f(0.05f, 0.05f, 0.05f);
		Box b; b.init(-d*.5,d*.5);
		bv.init(b);

		c->controller->m_targets.emplace_back(std::make_shared<IKTarget>(name,bv));
	}
}

void MotionRetargetScene::renderUI_autorig() {
	bool popState = m_showPop[POP_AR_RIGNET];
	if (popState) {
		// Always center this window when appearing
		ImVec2 center = ImGui::GetMainViewport()->GetCenter();
		ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

		if (ImGui::Begin("autorig rignet", &popState)) {
			ImGui::Text("make sure the char is facing z+");
			static ARrignetOptions options;
			ImGui::DragFloat("bandwidth",&options.bandwidth,0.01f,0.f,0.f,"%.10f");
			ImGui::DragFloat("threshold",&options.threshold,0.01f,0.f,0.f,"%.10f");
			if (ImGui::Button("reset options")) {
				options = ARrignetOptions();
			}
			ImGui::Checkbox("parse last output only", &options.parseOutputOnly);

			if (ImGui::Button("Confirm")) {
				
				ARrignet arr;
				arr.condaPath = m_settings.pathAnaconda;
				arr.rignetPath = m_settings.pathRignet;

				if (auto e = m_charEntityPrim.lock()) {
					arr.rig(&e->mesh,options);
					options = ARrignetOptions(); // reset params
					initCharacter(e);
				}
				
				popState = false;
			}
			ImGui::End();
		}
	}
	m_showPop[POP_AR_RIGNET] = popState;

	popState = m_showPop[POP_AR_PINOC];
	if (popState) {
		// Always center this window when appearing
		ImVec2 center = ImGui::GetMainViewport()->GetCenter();
		ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

		if (ImGui::Begin("autorig pinnocchio", &popState)) {
			if (ImGui::Button("Confirm")) {
				
				ARpinocchio arp;
				if (auto e = m_charEntityPrim.lock()) {
					arp.rig(&e->mesh,ARpinocchioOptions());
					initCharacter(e);
				}
				
				popState = false;
			}
			ImGui::End();
		}
	}
	m_showPop[POP_AR_PINOC] = popState;
}

void MotionRetargetScene::renderUI_autoMoRe() {
	bool popState = m_showPop[POP_MR_LIMB];
	static bool init = false;
	if (popState) {
		// Always center this window when appearing
		ImVec2 center = ImGui::GetMainViewport()->GetCenter();
		ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

		static std::vector<int> corr;

		if (ImGui::Begin("Motion Retarget Limb", &popState)) {

			auto ct = m_charEntityPrim.lock();
			auto cs = m_charEntitySec.lock();
			
			if (ct && cs) {

				auto& ctc = ct->controller;
				auto& csc = cs->controller;

				if (ctc && csc) {
					//if (corr.size() != ctc->m_ikArmature.m_jointChains.size())
					//	corr.resize(ctc->m_ikArmature.m_jointChains.size(),-1);

					//TODO(skade) limb matching initial guess
					if (corr.size() != ctc->m_ikArmature.m_jointChains.size()) {
						corr.resize(ctc->m_ikArmature.m_jointChains.size(),-1);
						for (int i = 0; i < corr.size(); ++i) {
							corr[i] = std::min(i, (int) csc->m_ikArmature.m_jointChains.size()-1);
						}
					}

					int i=0;
					for (auto& jct : ctc->m_ikArmature.m_jointChains) {
						
						std::vector<std::string> jcsNames;
						for (auto& jcs : csc->m_ikArmature.m_jointChains) {
							jcsNames.push_back(jcs.name);
						}
						ImGui::ComboStr(jct.name.c_str(),&corr[i],jcsNames);
						//ImGui::Select

						//if (ImGui::Selectable(chains[n].name.c_str(), is_selected)) {
						//	if (ImGui::GetIO().KeyCtrl) { // CTRL+click to toggle
						//		if (m_selChainIdx == n)
						//			m_selChainIdx = -1;
						//	}
						//	else
						//		m_selChainIdx = n;
						//}

						//if (ImGui::IsItemActive() && !ImGui::IsItemHovered()) {
						//	int n_next = n + (ImGui::GetMouseDragDelta(0).y < 0.f ? -1 : 1);
						//	if (n_next >= 0 && n_next < chains.size()) {
						//		std::swap(chains[n],chains[n_next]);
						//		ImGui::ResetMouseDragDelta();
						//	}
						//}

						//// Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
						//if (is_selected)
						//	ImGui::SetItemDefaultFocus();
						i++;
					}
				}
				else {
					ImGui::Text("make sure both characters are rigged");
				}
			}
			else {
				ImGui::Text("make sure to primary and secondary select characters");
			}
			if (ImGui::Button("Confirm")) {

				if (ct && cs) {
					//TODO(skade) only once
					//MRlimb mrLimb;
					//if (!init)
					//	mrLimb.initialize(cs->controller.get(),ct->controller.get());
					
					//m_MRlimb.initialize();
					m_MRlimb.initialize(cs,ct,corr);
				}
				corr.clear();
				popState = false;
			}
			ImGui::End();
		}
	}
	m_showPop[POP_MR_LIMB] = popState;
}

}//CForge
