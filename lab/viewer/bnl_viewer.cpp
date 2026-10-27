// Bullet NEXT Lab - viewer: two Lab engines side by side on the same scene (FL, CL.9).
// A HARFANG program. Opens straight into the scene (no splash screen).
//   bnl_viewer [--scene NAME] [--left ENGINE] [--right ENGINE] [--scenes DIR] [--assets DIR]
//              [--left-recording FILE] [--right-recording FILE] [--frames N]
// Engines: those linked in (legacy, next, nextf, nexts, jolt) and "replay" (needs a recording, e.g. one
// written by bn_harness_<backend> record or bnl_chrono). --frames N quits after N frames (checks).
//
// Mouse: left drag orbits, right drag pans, wheel zooms. Keys: Space play/pause, Right one frame,
// R restart, C contacts, 1-4 speed (1, 1/2, 1/4, 1/8), Escape quits.
//
// The same program is built for the browser (CL.10): one thread, the frame loop driven by the
// browser, the scenes and shaders packed into the page. When the scene folder has
// native_hashes.txt (written by bnl_run on the PC, single thread), the panel says whether each
// finished run reached the same final hash as the native build. Each finished run also prints
// "BNL_RESULT scene engine hash same|differs|-" for scripts; in the browser the page link can
// name the scene to open (...#pile_1000).
#include "engine/assets.h"
#include "engine/create_geometry.h"
#include "engine/dear_imgui.h"
#include "engine/render_pipeline.h"
#include "engine/scene.h"
#include "engine/scene_lab_physics.h"
#include "foundation/clock.h"
#include "foundation/format.h"
#include "foundation/log.h"
#include "foundation/matrix3.h"
#include "foundation/matrix44.h"
#include "foundation/projection.h"
#include "platform/input_system.h"
#include "platform/window_system.h"

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#include <emscripten/html5.h>
#endif

#include <imgui/imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <memory>
#include <string>
#include <vector>

#ifndef BN_SCENES_DIR
#define BN_SCENES_DIR "lab/scenes"
#endif

using namespace hg;

namespace {

struct Side {
	std::string engine, recording;
	std::unique_ptr<SceneLabPhysics> lab;
	std::vector<Model> models;
	std::vector<bool> soft;
	Model wheel;
	bool has_wheel = false;
	std::string error;

	void Load(const std::string &path, const bgfx::VertexLayout &layout) {
		Unload();
		lab.reset(new SceneLabPhysics);
		error.clear();
		// A recording is played back through the "replay" engine.
		#ifdef __EMSCRIPTEN__
		const int threads = 0; // the browser build has one thread
#else
		const int threads = -1;
#endif
		const bool ok = engine == "replay" ? LoadReplay(path) : lab->LoadScene(path, engine, threads);
		if (!ok) {
			error = lab->GetError();
			lab.reset();
			return;
		}
		for (int i = 0; i < lab->GetBodyCount(); ++i) {
			const bool is_soft = lab->GetBodyKind(i) == LBK_Soft;
			soft.push_back(is_soft);
			models.push_back(is_soft ? Model{} : lab->CreateBodyModel(i, layout));
		}
		if (lab->GetVehicleCount() > 0) {
			const float r = lab->GetWheelRadius(0, 0);
			wheel = CreateCylinderModel(layout, r, 0.5f * r, 24);
			has_wheel = true;
		}
	}

	bool LoadReplay(const std::string &path) { return lab->LoadRecording(path, recording); }

	void Unload() {
		for (auto &m : models)
			if (!m.lists.empty())
				Destroy(m);
		models.clear();
		soft.clear();
		if (has_wheel)
			Destroy(wheel);
		has_wheel = false;
		lab.reset();
	}
};

Color BodyColor(LabBodyKind k, int i) {
	switch (k) {
		case LBK_Static: return Color(0.3f, 0.31f, 0.33f, 1.f);
		case LBK_Sensor: return Color(0.3f, 0.8f, 0.4f, 0.35f);
		case LBK_Character: return Color(0.95f, 0.75f, 0.2f, 1.f);
		case LBK_Soft: return Color(0.85f, 0.35f, 0.7f, 1.f);
		default: break;
	}
	static const Color palette[] = {{0.26f, 0.52f, 0.96f, 1.f}, {0.95f, 0.45f, 0.25f, 1.f}, {0.3f, 0.75f, 0.55f, 1.f},
		{0.8f, 0.35f, 0.4f, 1.f}, {0.6f, 0.45f, 0.85f, 1.f}, {0.9f, 0.7f, 0.25f, 1.f}};
	return palette[i % 6];
}

// "scene engine hash" per line (lab/tools/native_hashes.py); empty if the file is missing.
std::map<std::string, std::string> LoadNativeHashes(const std::string &path) {
	std::map<std::string, std::string> h;
	std::ifstream in(path);
	std::string line;
	while (std::getline(in, line)) {
		std::istringstream ls(line);
		std::string scene, engine, hash;
		if (ls >> scene >> engine >> hash)
			h[scene + " " + engine] = hash;
	}
	return h;
}


struct App {
	std::string scenes_dir = BN_SCENES_DIR, scene_name = "stack_10";
	Side sides[2];
	int quit_after = -1, frames_drawn = 0;
	std::vector<std::string> scene_names, engine_names;
	std::map<std::string, std::string> native;
	Window *win = nullptr;
	int res_x = 1600, res_y = 900;
	bgfx::ProgramHandle lit = BGFX_INVALID_HANDLE, line = BGFX_INVALID_HANDLE;
	bgfx::VertexLayout layout, line_layout;
	std::unique_ptr<Vertices> soft_vtx, contact_vtx, grid_vtx;
	int scene_idx = 0;
	float yaw = 0.6f, pitch = 0.45f, dist = 22.f;
	Vec3 target = Vec3(0, 2, 0);
	bool playing = true, show_contacts = false;
	int speed_div = 1, sub_tick = 0;
	Mouse mouse;
	Keyboard keyboard;
	time_ns last = 0;

	int framed_scene = -1;

	void Load() {
		const std::string path = scenes_dir + "/" + scene_names[scene_idx] + ".json";
		for (auto &s : sides)
			s.Load(path, layout);
		if (framed_scene != scene_idx) { // a new scene: aim the camera at its moving bodies
			framed_scene = scene_idx;
			FrameScene();
		}
	}

	void FrameScene() { // every body's origin (the ground's is its centre), seen whole
		const SceneLabPhysics *lab = sides[0].lab ? sides[0].lab.get() : sides[1].lab.get();
		if (!lab || !lab->GetBodyCount())
			return;
		Vec3 lo(1e9f, 1e9f, 1e9f), hi(-1e9f, -1e9f, -1e9f);
		for (int i = 0; i < lab->GetBodyCount(); ++i) {
			const Vec3 p = GetT(lab->GetBodyWorld(i));
			lo = Min(lo, p);
			hi = Max(hi, p);
		}
		target = (lo + hi) * 0.5f;
		dist = std::max(8.f, Len(hi - lo) * 1.2f + 6.f);
	}

	// "matches the native build" / "differs from the native build" once a run is finished.
	std::string NativeCheck(const Side &sd) const {
		if (native.empty() || !sd.lab || sd.lab->GetFrame() < sd.lab->GetFrameCount())
			return "";
		const auto i = native.find(scene_names[scene_idx] + " " + sd.lab->GetEngineName());
		if (i == native.end())
			return "";
		return i->second == sd.lab->GetFinalHash() ? "same as native" : "DIFFERS from native " + i->second;
	}

	bool Frame();
};

App *app = nullptr;

bool App::Frame() {
	if (keyboard.Down(K_Escape) || !IsWindowOpen(win))
		return false;
#ifdef __EMSCRIPTEN__
	{ // follow the canvas size
		double cw = 0, ch = 0;
		emscripten_get_element_css_size("#canvas", &cw, &ch);
		if (cw > 0 && ch > 0 && (int(cw) != res_x || int(ch) != res_y)) {
			res_x = int(cw);
			res_y = int(ch);
			emscripten_set_canvas_element_size("#canvas", res_x, res_y);
		}
		bgfx::reset(res_x, res_y, BGFX_RESET_VSYNC);
	}
#else
	RenderResetToWindow(win, res_x, res_y, BGFX_RESET_VSYNC | BGFX_RESET_MSAA_X4);
#endif
	mouse.Update();
	keyboard.Update();
	const time_ns now = get_clock(), dt = now - last;
	last = now;

	ImGuiBeginFrame(res_x, res_y, dt, mouse.GetState(), keyboard.GetState());
	const bool ui = ImGui::GetIO().WantCaptureMouse;

	// Input
	if (!ui) {
		if (mouse.Down(MB_0)) {
			yaw -= mouse.DtX() * 0.005f;
			pitch = std::max(-1.4f, std::min(1.4f, pitch + mouse.DtY() * 0.005f));
		}
		if (mouse.Down(MB_1)) {
			const Vec3 right(std::cos(yaw), 0, -std::sin(yaw));
			target = target - right * (mouse.DtX() * dist * 0.0015f) + Vec3(0, mouse.DtY() * dist * 0.0015f, 0);
		}
		if (mouse.Wheel())
			dist = std::max(1.f, dist * (mouse.Wheel() > 0 ? 0.9f : 1.1f));
	}
	bool step_once = keyboard.Pressed(K_Right), restart = keyboard.Pressed(K_R);
	if (keyboard.Pressed(K_Space)) playing = !playing;
	if (keyboard.Pressed(K_C)) show_contacts = !show_contacts;
	if (keyboard.Pressed(K_1)) speed_div = 1;
	if (keyboard.Pressed(K_2)) speed_div = 2;
	if (keyboard.Pressed(K_3)) speed_div = 4;
	if (keyboard.Pressed(K_4)) speed_div = 8;

	// Panel
	// Top centre, over the seam between the two views: the scenes sit in the middle of each half.
	ImGui::SetNextWindowPos(ImVec2(0.5f * float(res_x), 8.f), ImGuiCond_Once, ImVec2(0.5f, 0.f));
	if (ImGui::Begin("Bullet NEXT Lab", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
		if (ImGui::BeginCombo("Scene", scene_names.empty() ? "-" : scene_names[scene_idx].c_str())) {
			for (int i = 0; i < int(scene_names.size()); ++i)
				if (ImGui::Selectable(scene_names[i].c_str(), i == scene_idx)) {
					scene_idx = i;
					restart = true;
				}
			ImGui::EndCombo();
		}
		for (int s = 0; s < 2; ++s) {
			ImGui::PushID(s);
			if (ImGui::BeginCombo(s ? "Right" : "Left", sides[s].engine.c_str())) {
				for (const auto &e : engine_names)
					if (ImGui::Selectable(e.c_str(), e == sides[s].engine)) {
						sides[s].engine = e;
						restart = true;
					}
				ImGui::EndCombo();
			}
			ImGui::PopID();
		}
		ImGui::Separator();
		if (ImGui::Button(playing ? "Pause" : "Play")) playing = !playing;
		ImGui::SameLine();
		if (ImGui::Button("Step")) step_once = true;
		ImGui::SameLine();
		if (ImGui::Button("Restart")) restart = true;
		ImGui::SliderInt("Slow motion", &speed_div, 1, 8, "1/%d");
		ImGui::Checkbox("Contacts", &show_contacts);
		ImGui::SameLine();
		ImGui::Text("   %.0f frames/s", ImGui::GetIO().Framerate);
		ImGui::Separator();
		for (int s = 0; s < 2; ++s) {
			const Side &sd = sides[s];
			ImGui::Text("%s: %s", s ? "Right" : "Left", sd.lab ? sd.lab->GetEngineName().c_str() : sd.engine.c_str());
			if (!sd.lab) {
				ImGui::TextColored(ImVec4(1, 0.5f, 0.4f, 1), "%s", sd.error.c_str());
				continue;
			}
			ImGui::Text("  frame %d / %d   step %.2f ms", sd.lab->GetFrame(), sd.lab->GetFrameCount(), sd.lab->GetLastStepMs());
			ImGui::Text("  hash %s   contacts %d", sd.lab->GetFinalHash().c_str(), sd.lab->GetContactCount());
			const std::string check = NativeCheck(sd);
			if (!check.empty())
				ImGui::TextColored(check[0] == 's' ? ImVec4(0.5f, 0.9f, 0.5f, 1) : ImVec4(1, 0.5f, 0.4f, 1), "  %s", check.c_str());
		}
	}
	ImGui::End();

	if (restart)
		Load();

	// Physics: one scene frame per display frame (60 Hz), slowed down by speed_div.
	const bool tick = (playing && (++sub_tick % speed_div) == 0) || step_once;
	if (tick)
		for (auto &s : sides)
			if (s.lab && s.lab->GetFrame() < s.lab->GetFrameCount()) {
				s.lab->Step();
				if (s.lab->GetFrame() == s.lab->GetFrameCount()) { // for scripts: one line per finished run
					const std::string check = NativeCheck(s);
					std::printf("BNL_RESULT %s %s %s %s\n", scene_names[scene_idx].c_str(), s.lab->GetEngineName().c_str(),
						s.lab->GetFinalHash().c_str(), check.empty() ? "-" : (check[0] == 's' ? "same" : "differs"));
					std::fflush(stdout);
				}
			}

	// Draw both halves
	const Vec3 eye = target + Vec3(std::sin(yaw) * std::cos(pitch), std::sin(pitch), std::cos(yaw) * std::cos(pitch)) * dist;
	const Mat4 cam = TransformationMat4(eye, Mat3LookAt(target - eye));
	const Vec3 light = Normalize(Vec3(-0.4f, -1.f, -0.3f));
	const int half = res_x / 2;
	for (int s = 0; s < 2; ++s) {
		const bgfx::ViewId view = bgfx::ViewId(s);
		SetViewPerspective(view, s * half, 0, half, res_y, cam, 0.05f, 500.f, 1.8f, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH,
			Color(0.12f, 0.13f, 0.15f, 1.f), 1.f, 0, Vec2(float(half) / float(res_y), 1.f));
		DrawLines(view, *grid_vtx, line);
		Side &sd = sides[s];
		if (!sd.lab)
			continue;
		const RenderState opaque = ComputeRenderState(BM_Opaque, DT_Less, FC_Disabled);
		const RenderState alpha = ComputeRenderState(BM_Alpha, DT_Less, FC_Disabled, false);
		for (int i = 0; i < sd.lab->GetBodyCount(); ++i) {
			const LabBodyKind kind = sd.lab->GetBodyKind(i);
			const Color c = BodyColor(kind, i);
			std::vector<UniformSetValue> values = {MakeUniformSetValue("uColor", Vec4(c.r, c.g, c.b, c.a)),
				MakeUniformSetValue("uLightDir", Vec4(light.x, light.y, light.z, 0.f))};
			if (sd.soft[i]) {
				sd.lab->GetSoftBodyTriangles(i, *soft_vtx, c);
				DrawTriangles(view, *soft_vtx, lit, values, {}, opaque);
				continue;
			}
			const Mat4 world = sd.lab->GetBodyWorld(i);
			DrawModel(view, sd.models[i], lit, values, {}, &world, 1, c.a < 1.f ? alpha : opaque);
		}
		if (sd.has_wheel)
			for (int v = 0; v < sd.lab->GetVehicleCount(); ++v)
				for (int w = 0; w < sd.lab->GetWheelCount(v); ++w) {
					// Bullet's wheel frame: axle along x; HARFANG's cylinder: along y.
					const Mat4 world = sd.lab->GetWheelWorld(v, w) * TransformationMat4(Vec3::Zero, Vec3(0, 0, Pi / 2));
					std::vector<UniformSetValue> values = {MakeUniformSetValue("uColor", Vec4(0.15f, 0.15f, 0.15f, 1.f)),
						MakeUniformSetValue("uLightDir", Vec4(light.x, light.y, light.z, 0.f))};
					DrawModel(view, sd.wheel, lit, values, {}, &world, 1, opaque);
				}
		if (show_contacts) {
			sd.lab->GetContactLines(*contact_vtx, 0.3f, Color(1.f, 0.85f, 0.2f, 1.f));
			if (contact_vtx->GetCount())
				DrawLines(view, *contact_vtx, line, ComputeRenderState(BM_Opaque, DT_Disabled, FC_Disabled));
		}
	}
	ImGuiEndFrame(255);
	bgfx::frame();
	UpdateWindow(win);
	return !(quit_after > 0 && ++frames_drawn >= quit_after);
}

#ifdef __EMSCRIPTEN__
void WebFrame() { app->Frame(); }
#endif

} // namespace

int main(int argc, char **argv) {
	app = new App;
	App &a = *app;
	std::string assets;
	a.sides[0].engine = "legacy";
	a.sides[1].engine = "nextf";
	for (int i = 1; i + 1 < argc; i += 2) {
		const std::string k = argv[i], v = argv[i + 1];
		if (k == "--scene") a.scene_name = v;
		else if (k == "--left") a.sides[0].engine = v;
		else if (k == "--right") a.sides[1].engine = v;
		else if (k == "--scenes") a.scenes_dir = v;
		else if (k == "--assets") assets = v;
		else if (k == "--left-recording") a.sides[0].recording = v;
		else if (k == "--right-recording") a.sides[1].recording = v;
		else if (k == "--frames") a.quit_after = std::atoi(v.c_str());
	}
#ifdef __EMSCRIPTEN__
	a.scenes_dir = "/scenes"; // packed into the page
	{ // the page link may name a scene: ...#pile_1000
		const char *hash = emscripten_run_script_string("window.location.hash.replace(/^#/, '')");
		if (hash && hash[0])
			a.scene_name = hash;
	}
	if (assets.empty())
		assets = "/assets_compiled";
#endif
	if (assets.empty()) // assets_compiled beside the program
		assets = std::string(argv[0]).substr(0, std::string(argv[0]).find_last_of("/\\") + 1) + "assets_compiled";
	AddAssetsFolder(assets.c_str());

	a.scene_names = SceneLabPhysics::GetSceneNames(a.scenes_dir);
	a.engine_names = SceneLabPhysics::GetEngineNames();
	a.engine_names.push_back("replay");
	a.native = LoadNativeHashes(a.scenes_dir + "/native_hashes.txt");

	InputInit();
	WindowSystemInit();
#ifdef __EMSCRIPTEN__
	double cw = 0, ch = 0;
	emscripten_get_element_css_size("#canvas", &cw, &ch);
	a.res_x = cw > 0 ? int(cw) : 1280;
	a.res_y = ch > 0 ? int(ch) : 720;
	a.win = NewWindow(a.res_x, a.res_y);
	if (!a.win || !RenderInit(a.win, bgfx::RendererType::OpenGLES)) {
		error("Could not start WebGL 2");
		return 1;
	}
	bgfx::reset(a.res_x, a.res_y, BGFX_RESET_VSYNC);
#else
	a.win = RenderInit("Bullet NEXT Lab", a.res_x, a.res_y, BGFX_RESET_VSYNC | BGFX_RESET_MSAA_X4);
	if (!a.win) {
		error("Could not open the window");
		return 1;
	}
#endif

	a.lit = LoadProgramFromAssets("shaders/lab_lit");
	a.line = LoadProgramFromAssets("shaders/lab_line");
	ImGuiInit(16.f, LoadProgramFromAssets("core/shader/imgui"), LoadProgramFromAssets("core/shader/imgui_image"));

	a.layout = VertexLayoutPosFloatNormFloat();
	a.line_layout = VertexLayoutPosFloatColorFloat();
	a.soft_vtx.reset(new Vertices(a.layout, 4096));
	a.contact_vtx.reset(new Vertices(a.line_layout, 4096));
	a.grid_vtx.reset(new Vertices(a.line_layout, 512));

	// Ground grid (HARFANG axes: Y up)
	{
		size_t n = 0;
		for (int i = -20; i <= 20; ++i) {
			const Color c = i == 0 ? Color(0.45f, 0.45f, 0.5f, 1.f) : Color(0.27f, 0.28f, 0.31f, 1.f);
			a.grid_vtx->Begin(n++).SetPos({float(i), 0.001f, -20.f}).SetColor0(c).End();
			a.grid_vtx->Begin(n++).SetPos({float(i), 0.001f, 20.f}).SetColor0(c).End();
			a.grid_vtx->Begin(n++).SetPos({-20.f, 0.001f, float(i)}).SetColor0(c).End();
			a.grid_vtx->Begin(n++).SetPos({20.f, 0.001f, float(i)}).SetColor0(c).End();
		}
	}

	a.scene_idx = int(std::find(a.scene_names.begin(), a.scene_names.end(), a.scene_name) - a.scene_names.begin());
	if (a.scene_idx >= int(a.scene_names.size()))
		a.scene_idx = 0;
	if (!a.scene_names.empty())
		a.Load();
	a.last = get_clock();

#ifdef __EMSCRIPTEN__
	emscripten_set_main_loop(WebFrame, 0, 1); // never returns
#else
	while (a.Frame()) {
	}
	for (auto &s : a.sides)
		s.Unload();
	ImGuiShutdown();
	RenderShutdown();
	DestroyWindow(a.win);
	WindowSystemShutdown();
	delete app;
#endif
	return 0;
}
